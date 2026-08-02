// ==UserScript==
// @name         Apollo 测评场景 Log 手动下载器
// @namespace    apollo-scenario-log-downloader
// @version      2.0
// @description  在测评详情页右下角显示浮动窗口：扫描回放链接(offlineview?id=xxx)
//               提取场景 id，每个场景一个【下载】按钮，手动点击才下载该系统日志(tgz)。
//               不自动下载。旧赛题可直下，无权限返回 403 FAILED_TO_AUTH。
// @author       mayaochang
// @match        https://apollo.baidu.com/community/competition/*
// @match        https://apollo.baidu.com/workspace-plus/*
// @grant        none
// @run-at       document-idle
// ==/UserScript==

(function () {
    'use strict';

    // ---------- 工具 ----------
    const LOG_API = '/api/workbench/tasks/scenario-logs/';

    function extractId(href) {
        if (!href) return null;
        const m = href.match(/[?&]id=([0-9a-fA-F]{24})/)
               || href.match(/scenario-logs\/([0-9a-fA-F]{24})/);
        return m ? m[1].toLowerCase() : null;
    }

    function esc(s) {
        return String(s).replace(/[&<>"']/g, (c) => ({
            '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'
        }[c]));
    }

    // 从回放链接附近的 DOM 尽量推断场景名
    function guessName(a) {
        // 1) 最近的 tr：取第一个看起来像名称的单元格
        const tr = a.closest('tr');
        if (tr) {
            const cells = tr.querySelectorAll('td');
            for (const c of cells) {
                const t = c.textContent.trim();
                if (t && t.length > 2 && t.length < 60 && !/^\d+$/.test(t)) {
                    return t;
                }
            }
        }
        // 2) 回退：URL 的 num 参数
        try {
            const num = new URL(a.href).searchParams.get('num');
            if (num) return num;
        } catch (e) {}
        // 3) 链接自身文本
        const t = a.textContent.trim();
        if (t) return t;
        return '';
    }

    // ---------- 扫描 ----------
    function scan() {
        const seen = new Map(); // id -> {name, href}
        document.querySelectorAll('a[href*="offlineview"], a[href*="scenario-logs"]')
            .forEach((a) => {
                const id = extractId(a.href);
                if (!id || seen.has(id)) return;
                seen.set(id, { name: guessName(a), href: a.href });
            });
        // 兜底：当前页面 URL
        const selfId = extractId(window.location.href);
        if (selfId && !seen.has(selfId)) {
            seen.set(selfId, { name: '当前页面', href: window.location.href });
        }
        return Array.from(seen.entries()); // [[id, {name, href}], ...]
    }

    // ---------- 下载 ----------
    async function downloadLog(id, btn, statusEl) {
        btn.disabled = true;
        statusEl.textContent = '下载中…';
        statusEl.style.color = '#888';
        try {
            const resp = await fetch(LOG_API + id, { credentials: 'include' });
            if (!resp.ok) {
                let body = '';
                try { body = (await resp.text()).slice(0, 80); } catch (e) {}
                statusEl.textContent = `❌ HTTP ${resp.status} ${body}`;
                statusEl.style.color = '#d33';
                return;
            }
            const cd = resp.headers.get('content-disposition') || '';
            const fnMatch = cd.match(/filename="?([^"]+)"?/i);
            const filename = (fnMatch && fnMatch[1]) || id + '.tgz';
            const blob = await resp.blob();
            const urlObj = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = urlObj;
            a.download = filename;
            document.body.appendChild(a);
            a.click();
            a.remove();
            setTimeout(() => URL.revokeObjectURL(urlObj), 15000);
            statusEl.textContent = `✅ ${filename} (${(blob.size / 1048576).toFixed(2)} MB)`;
            statusEl.style.color = '#1a7f37';
        } catch (e) {
            statusEl.textContent = `⚠️ ${e}`;
            statusEl.style.color = '#d33';
        } finally {
            btn.disabled = false;
        }
    }

    // ---------- 面板 ----------
    let panel = null;
    let bodyEl = null;
    let listEl = null;

    function buildPanel() {
        // 面板容器
        panel = document.createElement('div');
        panel.id = 'scnlog-panel';
        panel.style.cssText = [
            'position:fixed', 'right:16px', 'bottom:16px', 'z-index:2147483647',
            'width:340px', 'max-height:70vh', 'display:flex', 'flex-direction:column',
            'background:#1f2937', 'color:#e5e7eb', 'border:1px solid #374151',
            'border-radius:10px', 'box-shadow:0 8px 30px rgba(0,0,0,.45)',
            'font:12px/1.5 -apple-system,"Segoe UI",Roboto,"PingFang SC","Microsoft YaHei",sans-serif',
        ].join(';');

        // 头部
        const head = document.createElement('div');
        head.style.cssText = 'display:flex;align-items:center;gap:6px;padding:8px 10px;border-bottom:1px solid #374151;cursor:move;';
        head.innerHTML = '<span style="font-weight:700;flex:1">📦 场景 Log 下载器</span>';
        const refreshBtn = document.createElement('button');
        refreshBtn.textContent = '刷新';
        refreshBtn.title = '重新扫描页面回放链接';
        const foldBtn = document.createElement('button');
        foldBtn.textContent = '—';
        foldBtn.title = '折叠/展开';
        const closeBtn = document.createElement('button');
        closeBtn.textContent = '×';
        closeBtn.title = '关闭';
        [refreshBtn, foldBtn, closeBtn].forEach((b) => {
            b.style.cssText = 'background:#374151;color:#e5e7eb;border:none;border-radius:5px;padding:2px 8px;cursor:pointer;font-size:12px;';
        });
        head.appendChild(refreshBtn);
        head.appendChild(foldBtn);
        head.appendChild(closeBtn);
        panel.appendChild(head);

        // 列表
        bodyEl = document.createElement('div');
        bodyEl.style.cssText = 'overflow-y:auto;padding:8px;';
        listEl = document.createElement('div');
        bodyEl.appendChild(listEl);
        panel.appendChild(bodyEl);

        // 拖拽（简易）
        let dragging = false, dx = 0, dy = 0;
        head.addEventListener('mousedown', (e) => {
            dragging = true;
            dx = e.clientX - panel.offsetLeft;
            dy = e.clientY - panel.offsetTop;
            e.preventDefault();
        });
        document.addEventListener('mousemove', (e) => {
            if (!dragging) return;
            panel.style.left = (e.clientX - dx) + 'px';
            panel.style.top = (e.clientY - dy) + 'px';
            panel.style.right = 'auto';
            panel.style.bottom = 'auto';
        });
        document.addEventListener('mouseup', () => { dragging = false; });

        refreshBtn.addEventListener('click', render);
        foldBtn.addEventListener('click', () => {
            bodyEl.style.display = bodyEl.style.display === 'none' ? '' : 'none';
            foldBtn.textContent = bodyEl.style.display === 'none' ? '+' : '—';
        });
        closeBtn.addEventListener('click', () => { panel.remove(); panel = null; });

        document.body.appendChild(panel);
    }

    function render() {
        if (!panel) buildPanel();
        listEl.innerHTML = '';
        const items = scan();
        if (items.length === 0) {
            listEl.innerHTML = '<div style="color:#9ca3af;padding:6px">未发现回放链接(offlineview?id=...)</div>';
            return;
        }
        items.forEach(([id, info]) => {
            const row = document.createElement('div');
            row.style.cssText = 'display:flex;align-items:center;gap:6px;padding:6px;border-bottom:1px solid #374151;';
            const txt = document.createElement('div');
            txt.style.cssText = 'flex:1;min-width:0;';
            txt.innerHTML = `<div style="font-weight:600;overflow:hidden;text-overflow:ellipsis;white-space:nowrap">${esc(info.name || '未命名')}</div>` +
                `<div style="color:#9ca3af;font-size:11px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap">${esc(id)}</div>`;
            const btn = document.createElement('button');
            btn.textContent = '下载';
            btn.style.cssText = 'background:#2563eb;color:#fff;border:none;border-radius:5px;padding:4px 12px;cursor:pointer;font-size:12px;flex-shrink:0;';
            btn.addEventListener('click', () => downloadLog(id, btn, status));
            const status = document.createElement('div');
            status.style.cssText = 'flex:1;min-width:0;font-size:11px;color:#9ca3af;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;text-align:right;';
            // 两行布局：名称+id / 按钮+状态
            txt.style.flex = '1';
            const right = document.createElement('div');
            right.style.cssText = 'display:flex;flex-direction:column;align-items:flex-end;gap:4px;flex-shrink:0;';
            right.appendChild(btn);
            right.appendChild(status);
            row.appendChild(txt);
            row.appendChild(right);
            listEl.appendChild(row);
        });
    }

    // 启动：延迟等待页面表格渲染，再挂面板；简单监听一次动态内容
    setTimeout(render, 1200);
    let scanTimer = null;
    const mo = new MutationObserver(() => {
        clearTimeout(scanTimer);
        scanTimer = setTimeout(render, 1500);
    });
    mo.observe(document.documentElement, { childList: true, subtree: true });
})();
