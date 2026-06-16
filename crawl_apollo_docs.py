#!/usr/bin/env python3
"""
Apollo 文档爬取脚本
从 apollo.baidu.com 爬取所有文档并转换为 Markdown 文件
"""

import json
import os
import re
import time
import sys
from pathlib import Path
from urllib.parse import urljoin

import requests
from bs4 import BeautifulSoup

BASE_URL = "https://apollo.baidu.com/docs/apollo/latest/"
OUTPUT_DIR = Path("/home/skye/application-pnc/apollo_docs_md")
DOC_LIST_FILE = Path("/home/skye/application-pnc/doc_list.json")

HEADERS = {
    "User-Agent": "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/120.0.0.0 Safari/537.36",
    "Accept": "text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8",
    "Accept-Language": "zh-CN,zh;q=0.9,en;q=0.8",
}

# 需要跳过的文档（非内容页或外部链接）
SKIP_URLS = {
    "files.html",  # 源代码文档索引页
    "index.html",  # 首页（已作为 overview）
}

def sanitize_filename(name):
    """清理文件名，移除不合法字符"""
    name = re.sub(r'[<>:"/\\|?*]', '_', name)
    name = re.sub(r'\s+', '_', name)
    return name[:200]  # 限制长度

def html_to_markdown(soup_element, base_url=BASE_URL):
    """将 BeautifulSoup HTML 元素转换为 Markdown 文本"""
    md_lines = []
    
    for child in soup_element.children:
        if child.name is None:
            # 文本节点
            text = str(child).strip()
            if text:
                md_lines.append(text)
            continue
        
        tag = child.name
        
        if tag in ['h1', 'h2', 'h3', 'h4', 'h5', 'h6']:
            level = int(tag[1])
            text = child.get_text().strip()
            md_lines.append(f"\n{'#' * level} {text}\n")
        
        elif tag == 'p':
            text = process_inline_elements(child, base_url)
            if text.strip():
                md_lines.append(f"\n{text}\n")
        
        elif tag == 'pre':
            code_text = child.get_text()
            md_lines.append(f"\n```\n{code_text}\n```\n")
        
        elif tag == 'code':
            # 行内代码
            pass  # 由 process_inline_elements 处理
        
        elif tag in ['ul', 'ol']:
            for li in child.find_all('li', recursive=False):
                prefix = '- ' if tag == 'ul' else '1. '
                text = process_inline_elements(li, base_url)
                md_lines.append(f"{prefix}{text}")
            md_lines.append("")
        
        elif tag == 'table':
            md_lines.append(process_table(child, base_url))
        
        elif tag == 'dl':
            md_lines.append(process_definition_list(child, base_url))
        
        elif tag == 'div':
            cls = child.get('class', [])
            if 'note' in cls:
                text = process_inline_elements(child, base_url)
                md_lines.append(f"\n> **注意：** {text}\n")
            elif 'warning' in cls:
                text = process_inline_elements(child, base_url)
                md_lines.append(f"\n> **警告：** {text}\n")
            elif 'toc' in cls:
                pass  # 跳过目录
            elif 'block' in cls:
                text = process_inline_elements(child, base_url)
                md_lines.append(f"\n> {text}\n")
            elif 'image' in cls or 'imageblock' in cls:
                img = child.find('img')
                if img:
                    alt = img.get('alt', '')
                    src = img.get('src', '')
                    if src:
                        src = urljoin(base_url, src)
                    md_lines.append(f"\n![{alt}]({src})\n")
                text_content = child.get_text().strip()
                if text_content:
                    md_lines.append(text_content)
            elif 'level' in ''.join(cls):
                # 可能是一个层级的标题
                text = process_inline_elements(child, base_url)
                if text.strip():
                    md_lines.append(f"{text}")
            elif 'fragment' in ''.join(cls):
                md_lines.append(html_to_markdown(child, base_url))
            else:
                # 默认递归处理
                md_lines.append(html_to_markdown(child, base_url))
        
        elif tag == 'img':
            alt = child.get('alt', '')
            src = child.get('src', '')
            if src:
                src = urljoin(base_url, src)
            md_lines.append(f"\n![{alt}]({src})\n")
        
        elif tag == 'a':
            text = child.get_text().strip()
            href = child.get('href', '')
            if href and not href.startswith('#'):
                href = urljoin(base_url, href)
                md_lines.append(f"[{text}]({href})")
            else:
                md_lines.append(text)
        
        elif tag == 'br':
            md_lines.append("\n")
        
        elif tag == 'hr':
            md_lines.append("\n---\n")
        
        elif tag == 'blockquote':
            text = process_inline_elements(child, base_url)
            for line in text.split('\n'):
                md_lines.append(f"> {line}")
            md_lines.append("")
        
        else:
            # 递归处理其他元素
            md_lines.append(html_to_markdown(child, base_url))
    
    return '\n'.join(md_lines)


def process_inline_elements(element, base_url):
    """处理行内元素（strong, em, code, a 等）"""
    result = []
    for child in element.children:
        if child.name is None:
            result.append(str(child))
        elif child.name == 'strong' or child.name == 'b':
            result.append(f"**{child.get_text()}**")
        elif child.name == 'em' or child.name == 'i':
            result.append(f"*{child.get_text()}*")
        elif child.name == 'code':
            result.append(f"`{child.get_text()}`")
        elif child.name == 'a':
            text = child.get_text().strip()
            href = child.get('href', '')
            if href and not href.startswith('#'):
                href = urljoin(base_url, href)
                result.append(f"[{text}]({href})")
            else:
                result.append(text)
        elif child.name == 'img':
            alt = child.get('alt', '')
            src = child.get('src', '')
            if src:
                src = urljoin(base_url, src)
            result.append(f"![{alt}]({src})")
        elif child.name == 'br':
            result.append('\n')
        elif child.name == 'span':
            result.append(child.get_text())
        else:
            result.append(child.get_text())
    return ''.join(result)


def process_table(table_el, base_url):
    """将 HTML 表格转换为 Markdown 表格"""
    rows = table_el.find_all('tr')
    if not rows:
        return ""
    
    md_rows = []
    for row in rows:
        cells = row.find_all(['th', 'td'])
        md_cells = []
        for cell in cells:
            text = cell.get_text().strip().replace('\n', ' ').replace('|', '\\|')
            md_cells.append(text)
        md_rows.append(md_cells)
    
    if not md_rows:
        return ""
    
    # 构建 Markdown 表格
    col_count = max(len(r) for r in md_rows)
    normalized = [r + [''] * (col_count - len(r)) for r in md_rows]
    
    lines = []
    lines.append('| ' + ' | '.join(normalized[0]) + ' |')
    lines.append('| ' + ' | '.join(['---'] * col_count) + ' |')
    for row in normalized[1:]:
        lines.append('| ' + ' | '.join(row) + ' |')
    
    return '\n' + '\n'.join(lines) + '\n'


def process_definition_list(dl_el, base_url):
    """处理定义列表"""
    lines = []
    for child in dl_el.children:
        if child.name == 'dt':
            lines.append(f"\n**{child.get_text().strip()}**")
        elif child.name == 'dd':
            lines.append(f": {child.get_text().strip()}")
    return '\n'.join(lines) + '\n'


def extract_doc_content(html_content, url):
    """从 HTML 页面中提取文档内容"""
    soup = BeautifulSoup(html_content, 'lxml')
    
    # 尝试找到文档内容区域
    doc_content = soup.find('div', id='doc-content')
    if not doc_content:
        doc_content = soup.find('div', class_='doc-content')
    if not doc_content:
        doc_content = soup.find('div', class_='contents')
    if not doc_content:
        # 尝试 body
        doc_content = soup.find('body')
    
    if not doc_content:
        return None
    
    # 提取标题
    title_el = doc_content.find('div', class_='headertitle')
    title = ""
    if title_el:
        title_div = title_el.find('div', class_='title')
        if title_div:
            title = title_div.get_text().strip()
        title_el.decompose()  # 移除标题区域
    
    # 提取主内容
    contents_div = doc_content.find('div', class_='contents')
    if contents_div:
        content_el = contents_div
    else:
        content_el = doc_content
    
    # 移除 TOC 目录
    for toc in content_el.find_all('div', class_='toc'):
        toc.decompose()
    
    # 转换为 Markdown
    markdown = html_to_markdown(content_el, BASE_URL)
    
    # 清理多余空行
    markdown = re.sub(r'\n{3,}', '\n\n', markdown)
    markdown = markdown.strip()
    
    return title, markdown


def fetch_doc(url):
    """获取文档页面内容"""
    full_url = urljoin(BASE_URL, url)
    try:
        resp = requests.get(full_url, headers=HEADERS, timeout=30)
        resp.encoding = 'utf-8'
        if resp.status_code == 200:
            return resp.text
        else:
            print(f"  HTTP {resp.status_code}: {full_url}")
            return None
    except Exception as e:
        print(f"  请求失败: {full_url} - {e}")
        return None


def save_markdown(doc_info, title, markdown, output_dir):
    """保存 Markdown 文件"""
    # 根据路径创建目录结构
    path_parts = doc_info['path'].split(' > ')
    
    # 去除最后一个（就是文档自身的标题），用前面的作为目录
    if len(path_parts) > 1:
        folder_parts = path_parts[:-1]
        file_name = sanitize_filename(path_parts[-1]) + '.md'
    else:
        folder_parts = []
        file_name = sanitize_filename(title or doc_info['title']) + '.md'
    
    # 创建目录
    if folder_parts:
        dir_path = output_dir.joinpath(*[sanitize_filename(p) for p in folder_parts])
    else:
        dir_path = output_dir
    
    dir_path.mkdir(parents=True, exist_ok=True)
    
    # 写入文件
    file_path = dir_path / file_name
    
    # 构建带元数据的 Markdown
    full_md = f"---\ntitle: {title or doc_info['title']}\n"
    full_md += f"source: {urljoin(BASE_URL, doc_info['url'])}\n"
    full_md += f"category: {doc_info['path']}\n---\n\n"
    
    if title:
        full_md += f"# {title}\n\n"
    
    full_md += markdown
    
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(full_md)
    
    return file_path


def main():
    print("=" * 60)
    print("Apollo 文档爬取工具")
    print("=" * 60)
    
    # 读取文档列表
    with open(DOC_LIST_FILE, 'r', encoding='utf-8') as f:
        doc_list = json.load(f)
    
    total = len(doc_list)
    print(f"\n共发现 {total} 个文档链接\n")
    
    # 去重（按 URL）
    seen_urls = set()
    unique_docs = []
    for doc in doc_list:
        if doc['url'] not in seen_urls and doc['url'] not in SKIP_URLS:
            seen_urls.add(doc['url'])
            unique_docs.append(doc)
    
    print(f"去重后共 {len(unique_docs)} 个文档（已跳过 {total - len(unique_docs)} 个）\n")
    
    # 清理输出目录
    if OUTPUT_DIR.exists():
        import shutil
        shutil.rmtree(OUTPUT_DIR)
    
    success_count = 0
    fail_count = 0
    
    for i, doc in enumerate(unique_docs, 1):
        url = doc['url']
        title = doc['title']
        path = doc['path']
        
        print(f"[{i}/{len(unique_docs)}] {title}")
        print(f"  路径: {path}")
        print(f"  URL: {url}")
        
        html = fetch_doc(url)
        if html:
            doc_title, markdown = extract_doc_content(html, url)
            if markdown:
                file_path = save_markdown(doc, doc_title, markdown, OUTPUT_DIR)
                print(f"  ✓ 已保存: {file_path}")
                success_count += 1
            else:
                print(f"  ✗ 无法提取内容")
                fail_count += 1
        else:
            fail_count += 1
        
        # 礼貌延迟，避免请求过快
        time.sleep(0.5)
        
        if i % 20 == 0:
            print(f"\n--- 进度: {i}/{len(unique_docs)}, 成功: {success_count}, 失败: {fail_count} ---\n")
    
    print("\n" + "=" * 60)
    print(f"爬取完成！")
    print(f"  成功: {success_count}")
    print(f"  失败: {fail_count}")
    print(f"  输出目录: {OUTPUT_DIR}")
    print("=" * 60)


if __name__ == '__main__':
    main()
