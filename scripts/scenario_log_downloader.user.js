// ==UserScript==
// @name         Apollo 测评场景 Log 手动下载器
// @namespace    apollo-scenario-log-downloader
// @version      2.2
// @description  在测评详情页右下角显示浮动窗口：扫描回放链接(offlineview?id=xxx)
//               提取场景 id，每个场景一个【下载】+【诊断】按钮，手动点击才下载该系统
//               日志(tgz)。不自动下载。旧赛题可直下，无权限返回 403 FAILED_TO_AUTH。
//               诊断：检查 Cookie / 网络 / 接口状态码并给出建议。
//               fetch 失败(TypeError)自动降级浏览器原生下载，绕过 JS 层拦截。
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
        } catch (e) { }
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
    // fetch 失败时降级：浏览器原生导航下载（不经过 JS fetch，绕过扩展/插件拦截）
    function nativeDownload(url) {
        try {
            const a = document.createElement('a');
            a.href = url;
            a.download = '';
            a.style.display = 'none';
            document.body.appendChild(a);
            a.click();
            setTimeout(() => a.remove(), 5000);
        } catch (e) {
            try { window.open(url, '_blank'); } catch (e2) { }
        }
    }

    async function downloadLog(id, btn, statusEl) {
        btn.disabled = true;
        statusEl.textContent = '下载中…';
        statusEl.style.color = '#888';
        const url = LOG_API + id;
        try {
            const resp = await fetch(url, { credentials: 'include' });
            if (!resp.ok) {
                let body = '';
                try { body = (await resp.text()).slice(0, 80); } catch (e) { }
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
            const name = (e && e.name) || 'Error';
            statusEl.textContent = `⚠️ ${name}，改用浏览器原生下载…`;
            statusEl.style.color = '#d33';
            nativeDownload(url);
        } finally {
            btn.disabled = false;
        }
    }

    // ---------- 诊断 ----------
    function statusHint(status) {
        if (status === 200 || status === 206) return { ok: true, tip: '接口可达且允许下载 ✓' };
        if (status === 401) return { ok: false, tip: '未登录（401）。请先登录 apollo.baidu.com 再试。' };
        if (status === 403) return { ok: false, tip: '无权限（403 FAILED_TO_AUTH）。登录态可能失效，或该赛题记录已无下载权限。重新登录后重试；仍 403 则说明该记录无权下载。' };
        if (status === 404) return { ok: false, tip: '记录不存在（404）。id 可能无效或接口路径有误。' };
        if (status === 405) return { ok: false, tip: '方法不允许（405）。接口不支持当前请求方式。' };
        if (status === 429) return { ok: false, tip: '请求太频繁（429）。稍等片刻再试。' };
        if (status >= 500) return { ok: false, tip: `服务端错误（${status}）。服务器异常，稍后重试。` };
        return { ok: false, tip: `HTTP ${status}。` };
    }

    async function diagnoseLog(id, info) {
        const rows = []; // {k, v, cls}  cls: '' | 'ok' | 'bad' | 'warn'
        const add = (k, v, cls) => rows.push({ k, v, cls: cls || '' });

        add('页面 URL', location.href, '');
        add('场景 id', id, '');
        add('场景名', (info && info.name) ? info.name : '-', '');
        const cookies = document.cookie.split(';').map(s => s.trim().split('=')[0]).filter(Boolean);
        add('可见 Cookie (' + cookies.length + ')',
            cookies.length ? cookies.join('、') : '无（登录态可能在 httpOnly，JS 不可见）',
            cookies.length ? 'ok' : 'warn');

        const url = LOG_API + id;
        add('请求', 'GET ' + url, '');
        let resp = null;
        try {
            // 用 Range 只拉 1 字节做探测，避免下载整个大文件
            resp = await fetch(url, { credentials: 'include', headers: { 'Range': 'bytes=0-0' } });
        } catch (e) {
            const name = (e && e.name) || 'Error';
            const msg = (e && e.message) || String(e);
            add('网络异常', name + ': ' + msg, 'bad');

            // 环境排查：Service Worker / CSP
            try {
                add('Service Worker',
                    navigator.serviceWorker.controller ? '页面被 SW 控制(可能拦截请求)' : '无 SW 控制',
                    navigator.serviceWorker.controller ? 'warn' : 'ok');
            } catch (err) { }
            const cspMeta = document.querySelector('meta[http-equiv="Content-Security-Policy" i]');
            if (cspMeta) add('页面 CSP', cspMeta.getAttribute('content') || '(有 meta CSP)', 'warn');

            // XHR 兜底探测：XHR 成功 → fetch 被特定机制拦截；XHR 也失败 → 网络/插件
            let xhrOk = false, xhrInfo = '';
            try {
                const xhrResp = await new Promise((resolve, reject) => {
                    const x = new XMLHttpRequest();
                    x.open('GET', url, true);
                    x.withCredentials = true;
                    x.onload = () => resolve({ status: x.status, len: (x.responseText || '').length });
                    x.onerror = () => reject(new Error('xhr onerror status=' + x.status));
                    x.send();
                });
                xhrOk = true;
                xhrInfo = 'XHR 可达 (HTTP ' + xhrResp.status + ', ' + xhrResp.len + 'B)';
            } catch (err) {
                xhrInfo = 'XHR 也失败: ' + ((err && err.message) || err);
            }
            add('XHR 兜底', xhrInfo, xhrOk ? 'ok' : 'bad');

            let guess;
            if (xhrOk) {
                guess = 'fetch 被拦截但 XHR 可达 → 常见原因是浏览器扩展(广告拦截/隐私插件)按规则拦截了 fetch/api 请求，或页面脚本干扰。';
            } else if (/Failed to fetch|NetworkError|Network request failed/i.test(msg)) {
                guess = '浏览器无法到达服务器：网络中断 / 跨域(CORS)被拦截 / 广告拦截插件拦截 / 站点不可达。';
            } else if (/Load failed/i.test(msg)) {
                guess = '连接建立后中断：服务器主动断开或请求被拦截。';
            } else {
                guess = '未知网络错误。按 F12 → Network 查看该请求详情。';
            }
            add('判断', guess, 'bad');
            add('建议', '1) 关闭广告拦截/隐私插件(如 uBlock/AdBlock)后重试；2) F12 控制台看是否有 CORS 报错；3) F12 → Network 看请求是否发出、被谁中止；4) 直接浏览器访问 ' + url + ' 看能否下载。', 'warn');
            showDiagnose(id, rows);
            return;
        }

        add('状态', resp.status + ' ' + resp.statusText, resp.ok ? 'ok' : 'bad');
        const h = statusHint(resp.status);
        add('判断', h.tip, h.ok ? 'ok' : 'bad');

        const ct = resp.headers.get('content-type');
        const cd = resp.headers.get('content-disposition');
        const cr = resp.headers.get('content-range');
        const cl = resp.headers.get('content-length');
        if (ct) add('Content-Type', ct, '');
        if (cr) add('Content-Range', cr, '');
        if (cl) add('Content-Length', cl, '');
        if (cd) add('Content-Disposition', cd, '');

        // 只在失败或响应体很小时读取正文；200 全量响应直接中断，避免拉大文件
        const full200 = resp.ok && resp.status === 200;
        if (full200) {
            try { resp.body && resp.body.cancel(); } catch (err) { }
            add('响应体', '（200 全量响应，未读取正文以免下载大文件；直接点【下载】即可）', 'ok');
        } else if (!resp.ok || parseInt(resp.headers.get('content-length') || '0', 10) < 8192) {
            let body = '';
            try { body = (await resp.text()).slice(0, 200); } catch (err) { }
            if (body) add('响应体(前200)', body, '');
        } else {
            try { resp.body && resp.body.cancel(); } catch (err) { }
        }

        if (resp.status === 206 && cr) {
            const m = cr.match(/\/(\d+)\s*$/);
            if (m) add('文件总大小', (parseInt(m[1], 10) / 1048576).toFixed(2) + ' MB', 'ok');
        }
        if (!resp.ok) {
            add('建议', '1) 403 → 先重新登录 apollo.baidu.com 再点【下载】；仍失败则该记录无权限。2) 5xx → 服务器问题，稍后重试。3) 其他 → 按 F12 Network 面板核对请求与响应。', 'warn');
        } else {
            add('建议', '接口可达且允许下载，直接点【下载】即可。', 'ok');
        }
        showDiagnose(id, rows);
    }

    // 诊断报告弹窗
    function showDiagnose(id, rows) {
        const mask = document.createElement('div');
        mask.style.cssText = 'position:fixed;inset:0;z-index:2147483646;background:rgba(0,0,0,.5);display:flex;align-items:center;justify-content:center;';
        const box = document.createElement('div');
        box.style.cssText = 'width:540px;max-width:92vw;max-height:80vh;overflow:auto;background:#111827;color:#e5e7eb;border:1px solid #374151;border-radius:10px;box-shadow:0 8px 30px rgba(0,0,0,.5);font:12px/1.6 -apple-system,"Segoe UI",Roboto,"PingFang SC","Microsoft YaHei",sans-serif;';
        let html = `<div style="padding:10px 14px;border-bottom:1px solid #374151;font-weight:700;font-size:13px;">🔍 诊断报告 <span style="color:#9ca3af;font-weight:400">${esc(id)}</span></div>`;
        html += '<div style="padding:6px 14px;">';
        for (const r of rows) {
            const color = r.cls === 'ok' ? '#34d399' : r.cls === 'bad' ? '#f87171' : r.cls === 'warn' ? '#fbbf24' : '#9ca3af';
            html += `<div style="padding:3px 0;border-bottom:1px dashed #1f2937;word-break:break-all;"><span style="color:#9ca3af;min-width:130px;display:inline-block;">${esc(r.k)}</span><span style="color:${color}">${esc(r.v)}</span></div>`;
        }
        html += '</div>';
        const foot = document.createElement('div');
        foot.style.cssText = 'padding:8px 14px;text-align:right;border-top:1px solid #374151;';
        const closeBtn = document.createElement('button');
        closeBtn.textContent = '关闭';
        closeBtn.style.cssText = 'background:#374151;color:#e5e7eb;border:none;border-radius:5px;padding:4px 14px;cursor:pointer;font-size:12px;';
        closeBtn.addEventListener('click', () => mask.remove());
        foot.appendChild(closeBtn);
        box.innerHTML = html;
        box.appendChild(foot);
        mask.appendChild(box);
        mask.addEventListener('click', (e) => { if (e.target === mask) mask.remove(); });
        document.body.appendChild(mask);
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
            const diagBtn = document.createElement('button');
            diagBtn.textContent = '诊断';
            diagBtn.title = '诊断该场景下载问题（Cookie/网络/状态码）';
            diagBtn.style.cssText = 'background:#4b5563;color:#e5e7eb;border:none;border-radius:5px;padding:4px 10px;cursor:pointer;font-size:12px;flex-shrink:0;';
            diagBtn.addEventListener('click', () => diagnoseLog(id, info));
            const status = document.createElement('div');
            status.style.cssText = 'flex:1;min-width:0;font-size:11px;color:#9ca3af;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;text-align:right;';
            // 两行布局：名称+id / 按钮+状态
            txt.style.flex = '1';
            const right = document.createElement('div');
            right.style.cssText = 'display:flex;flex-direction:column;align-items:flex-end;gap:4px;flex-shrink:0;';
            const btnRow = document.createElement('div');
            btnRow.style.cssText = 'display:flex;gap:4px;align-items:center;';
            btnRow.appendChild(btn);
            btnRow.appendChild(diagBtn);
            right.appendChild(btnRow);
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
