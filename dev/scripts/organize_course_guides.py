"""Organize a full course-guide export (all stages) into an offline archive.

Usage: python3 -I dev/scripts/organize_course_guides.py <export.json> <output-dir> [<older-export.json>]

The export is produced in a logged-in browser session; no cookies are stored or needed here.
Standard library only, so it can run isolated (-I) on downloaded content.
"""
import concurrent.futures
import hashlib
import html
import json
import os
import pathlib
import re
import shutil
import sys
import urllib.parse
import urllib.request
from html.parser import HTMLParser

ROOT = pathlib.Path(__file__).resolve().parents[2]
EXPORT = pathlib.Path(sys.argv[1]).resolve()
OUT = pathlib.Path(sys.argv[2]).resolve()
OLDER = pathlib.Path(sys.argv[3]).resolve() if len(sys.argv) > 3 else None
BASE = 'http://10.126.59.136:30060'
IMAGE_HOSTS = ('10.126.59.136', 'tecs-prod-static.obs.cn-north-1.myhuaweicloud.com', 'www.hiascend.com')
VIDEO_ROOT = ROOT / '资料/原始解压/软件开发与实训综合实践资料分享/基于计算机视觉的智能机械臂应用实践/视频'
OLD_ASSETS = ROOT / '资料/课程说明/第二阶段-机械臂基础控制/assets'
KEYS = [('introduction', '任务描述'), ('step', '任务指导'), ('solution', '任务实现')]
VOID = {'area', 'base', 'br', 'col', 'embed', 'hr', 'img', 'input', 'link', 'meta', 'source', 'track', 'wbr'}
DROP = {'script', 'object', 'embed', 'style'}
BLOCK = {'p', 'div', 'br', 'li', 'tr', 'pre', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'table', 'figure', 'blockquote'}

export = json.loads(EXPORT.read_text(encoding='utf-8-sig'))
SOURCE = export['source']
guides = export['guides']
(OUT / 'assets').mkdir(parents=True, exist_ok=True)
(OUT / '代码片段').mkdir(exist_ok=True)
if EXPORT.parent != OUT:
    shutil.copyfile(EXPORT, OUT / '课程原始内容.json')


class Collector(HTMLParser):
    """First pass: collect image, iframe and video references."""

    def __init__(self):
        super().__init__()
        self.images, self.iframes, self.videos = set(), [], set()

    def handle_starttag(self, tag, attrs):
        a = dict(attrs)
        if tag == 'img' and a.get('src'):
            self.images.add(urllib.parse.urljoin(BASE, a['src']))
        elif tag == 'iframe' and a.get('src'):
            self.iframes.append((urllib.parse.urljoin(BASE, a['src']), a.get('title') or ''))
        elif tag in ('video', 'source') and a.get('src'):
            self.videos.add(a['src'])

    handle_startendtag = handle_starttag


collector = Collector()
for g in guides:
    for key, _ in KEYS:
        collector.feed(g['detail'].get(key) or '')


def fetch(url, target, accept):
    if not target.exists():
        with urllib.request.urlopen(url, timeout=30) as response:
            content_type = response.headers.get_content_type()
            if not accept(content_type):
                raise ValueError('Unexpected content type ' + content_type)
            target.write_bytes(response.read())


def download_image(url):
    parsed = urllib.parse.urlparse(url)
    if parsed.hostname not in IMAGE_HOSTS:
        return {'url': url, 'status': 'external-skipped'}
    suffix = pathlib.Path(urllib.parse.unquote(parsed.path)).suffix.lower()
    name = hashlib.sha256(url.encode()).hexdigest()[:20] + (suffix if suffix in ('.png', '.jpg', '.jpeg', '.gif', '.svg') else '.img')
    target = OUT / 'assets' / name
    try:
        reused = False
        if not target.exists() and (OLD_ASSETS / name).is_file():
            shutil.copyfile(OLD_ASSETS / name, target)
            reused = True
        fetch(url, target, lambda ct: ct.startswith('image/'))
        return {'url': url, 'status': 'downloaded', 'local': 'assets/' + name, 'bytes': target.stat().st_size, 'reusedFromOldCapture': reused}
    except Exception as exc:
        return {'url': url, 'status': 'failed', 'error': str(exc)}


with concurrent.futures.ThreadPoolExecutor(max_workers=6) as executor:
    resources = list(executor.map(download_image, sorted(collector.images)))
mapping = {r['url']: r['local'] for r in resources if r['status'] == 'downloaded'}

for url, title in collector.iframes:
    local = VIDEO_ROOT / title
    if title and local.is_file():
        mapping[url] = pathlib.Path(os.path.relpath(local, OUT)).as_posix()
        entry = {'url': url, 'status': 'linked-local-video', 'title': title, 'local': mapping[url]}
    else:
        entry = {'url': url, 'status': 'online-only', 'title': title}
    if entry not in resources:
        resources.append(entry)
for url in sorted(collector.videos):
    if urllib.parse.urlparse(url).hostname != 'staticfile.eduplus.net':
        resources.append({'url': url, 'status': 'external-skipped'})
        continue
    name = hashlib.sha256(url.encode()).hexdigest()[:20] + '.mp4'
    target = OUT / 'assets' / name
    try:
        if not target.exists() and (OLD_ASSETS / name).is_file():
            shutil.copyfile(OLD_ASSETS / name, target)
        fetch(url, target, lambda ct: ct in ('video/mp4', 'application/octet-stream'))
        mapping[url] = 'assets/' + name
        resources.append({'url': url, 'status': 'downloaded', 'local': mapping[url], 'bytes': target.stat().st_size})
    except Exception as exc:
        resources.append({'url': url, 'status': 'failed', 'error': str(exc)})
(OUT / '资源下载清单.json').write_text(json.dumps(resources, ensure_ascii=False, indent=2), encoding='utf-8')


class Rewriter(HTMLParser):
    """Second pass: drop active content, localize links, collect text and code blocks."""

    def __init__(self):
        super().__init__()
        self.out, self.text, self.codes = [], [], []
        self.skip = 0
        self.pre = None

    def _attrs(self, attrs):
        kept = []
        for name, value in attrs:
            if name.lower().startswith('on'):
                continue
            if name in ('src', 'href') and value is not None:
                if value.strip().lower().startswith('javascript:'):
                    continue
                absolute = urllib.parse.urljoin(BASE, value)
                value = mapping.get(value, mapping.get(absolute, absolute))
            kept.append(name if value is None else f'{name}="{html.escape(value, quote=True)}"')
        return (' ' + ' '.join(kept)) if kept else ''

    def handle_starttag(self, tag, attrs):
        if self.skip:
            if tag not in VOID:
                self.skip += 1
            return
        if tag in DROP:
            if tag not in VOID:
                self.skip = 1
            return
        if tag == 'iframe':
            a = dict(attrs)
            absolute = urllib.parse.urljoin(BASE, a.get('src') or SOURCE)
            href = mapping.get(absolute, absolute)
            label = (a.get('title') or '课程嵌入资源') + ('（在线预览）' if href.startswith('http') else '（本地视频）')
            self.out.append(f'<a href="{html.escape(href, quote=True)}">{html.escape(label)}</a>')
            self.text.append(label)
            self.skip = 1
            return
        if tag == 'pre':
            self.pre = []
        if tag in BLOCK:
            self.text.append('\n')
        self.out.append(f'<{tag}{self._attrs(attrs)}>')

    def handle_startendtag(self, tag, attrs):
        if self.skip or tag in DROP:
            return
        if tag in BLOCK:
            self.text.append('\n')
        self.out.append(f'<{tag}{self._attrs(attrs)}>')

    def handle_endtag(self, tag):
        if self.skip:
            self.skip -= 1
            return
        if tag in VOID:
            return
        if tag == 'pre' and self.pre is not None:
            self.codes.append(''.join(self.pre).replace('\xa0', ' '))
            self.pre = None
        if tag in BLOCK:
            self.text.append('\n')
        self.out.append(f'</{tag}>')

    def handle_data(self, data):
        if self.skip:
            return
        self.out.append(html.escape(data, quote=False))
        self.text.append(data)
        if self.pre is not None:
            self.pre.append(data)


def safe(name):
    return re.sub(r'[\\/:*?"<>|]', '-', name)


older = {}
if OLDER:
    for task in json.loads(OLDER.read_text(encoding='utf-8-sig')):
        older[task['id']] = task

style = 'body{font:16px/1.8 system-ui,sans-serif;max-width:1000px;margin:32px auto;padding:0 24px;color:#202124}img{max-width:100%;height:auto}table{border-collapse:collapse;max-width:100%;display:block;overflow:auto}td,th{border:1px solid #ccc;padding:8px}pre{white-space:pre-wrap;background:#f5f6f7;padding:16px;overflow:auto}a{color:#1558b0}.note{background:#fff6de;padding:12px}figure{margin:20px 0}'
stage_ids = []
for g in guides:
    if g['stageId'] not in stage_ids:
        stage_ids.append(g['stageId'])
counters = {}
readme = ['# 实训指导离线归档', '', '来源：' + SOURCE, '', '采集时间（UTC）：' + export['capturedAt'], '',
          '保存采集时学生可见的全部任务。HTML 含已下载配图和课程公开代码；示例未连接硬件运行，不代表本机实测通过。', '']
index_items, changes, last_stage = [], [], None
for g in guides:
    task = g['detail']
    stage_no = stage_ids.index(g['stageId']) + 1
    counters[stage_no] = counters.get(stage_no, 0) + 1
    prefix = f'{stage_no}-{counters[stage_no]:02d}'
    stem = prefix + '-' + safe(task['name'])
    if g['stage'] != last_stage:
        readme += ['', '## ' + g['stage'], '', '|任务|工时|说明|代码块|任务实现|相对旧采集|', '|---|---|---|---|---|---|']
        index_items.append('</ol><h2>' + html.escape(g['stage']) + '</h2><ol>')
        last_stage = g['stage']
    sections, text_parts, code_parts = [], [], []
    for key, title in KEYS:
        rewriter = Rewriter()
        rewriter.feed(task.get(key) or '')
        rewriter.close()
        for block_number, code in enumerate(rewriter.codes, 1):
            code_parts.append(f'## {title} · 代码块{block_number}\n\n```\n{code}\n```\n')
            # Keep independent blocks independent, preserving the course's execution order.
            if re.search(r'(?m)^\s*(?:import |from |def |class )', code):
                (OUT / '代码片段' / f'{prefix}-{key}-{block_number:02d}.py').write_text(
                    '# 课程原始代码片段；须结合实验说明按顺序执行，未进行硬件验证。\n' + code + '\n', encoding='utf-8')
        body = ''.join(rewriter.out)
        sections.append('<h2>' + title + '</h2>' + (body if task.get(key) else '<p>该字段为空（未向学生开放或无内容）。</p>'))
        text_parts.append(title + '\n\n' + re.sub(r'\n{3,}', '\n\n', ''.join(rewriter.text)).strip())
    doc = ('<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>'
           + html.escape(task['name']) + '</title><style>' + style + '</style></head><body><a href="index.html">返回目录</a><h1>' + html.escape(task['name'])
           + '</h1><p>' + html.escape(g['stage']) + ' · 课程建议工时：' + str(task['taskHour'])
           + ' 小时</p><p class="note">课程资料离线归档。代码按原文保留，尚未进行设备运行验证。图片已本地化，视频链接复用已有本地文件。</p>'
           + ''.join(sections) + '</body></html>')
    (OUT / (stem + '.html')).write_text(doc, encoding='utf-8-sig')
    (OUT / (stem + '.txt')).write_text(task['name'] + '\n\n' + '\n\n'.join(text_parts) + '\n', encoding='utf-8-sig')
    code_link = '—'
    if code_parts:
        (OUT / '代码片段' / (stem + '.md')).write_text(
            '# ' + task['name'] + '\n\n代码块保持原文及顺序，包含环境命令与Python片段，不能整体作为一个Python文件运行。\n\n' + '\n'.join(code_parts), encoding='utf-8-sig')
        code_link = f'[原始代码块](代码片段/{stem}.md)'
    solution = '有' if task.get('solution') else ('未开放' if task.get('solutionVisible') == 'No' else '空')
    if not older:
        status = '—'
    elif task['id'] not in older:
        status = '新增'
    else:
        changed = [title for key, title in KEYS if (older[task['id']].get(key) or '') != (task.get(key) or '')]
        status = ('有改动：' + '、'.join(changed)) if changed else '未变'
    changes.append((task['name'], status))
    readme.append(f'|{task["name"]}|{task["taskHour"]}|[图文说明]({stem}.html)|{code_link}|{solution}|{status}|')
    index_items.append('<li><a href="' + html.escape(stem + '.html', quote=True) + '">' + html.escape(task['name']) + '</a></li>')

image_results = [r for r in resources if r['url'] in collector.images]
readme += ['', '## 资源', '',
           f'配图下载：{sum(r["status"] == "downloaded" for r in image_results)}/{len(collector.images)}。详见[资源下载清单](资源下载清单.json)。', '',
           '嵌入视频（iframe）不重复下载：标题与原始解压目录中视频同名的，链接指向该本地文件；其余保留在线预览链接。', '',
           '“任务实现”列为“未开放”的任务，平台对学生隐藏了参考实现（solutionVisible=No），并非采集遗漏。']
(OUT / 'README.md').write_text('\n'.join(readme) + '\n', encoding='utf-8-sig')
(OUT / 'index.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>实训指导目录</title><style>' + style
                                + '</style><h1>实训指导离线归档</h1><p>采集时间（UTC）：' + html.escape(export['capturedAt']) + '。示例尚未连接硬件运行。</p><ol>'
                                + ''.join(index_items) + '</ol></html>', encoding='utf-8-sig')
print(json.dumps({'tasks': len(guides), 'images': sum(r['status'] == 'downloaded' for r in image_results), 'image_total': len(collector.images),
                  'not_downloaded': [r for r in resources if r['status'] not in ('downloaded', 'linked-local-video')], 'changes': changes}, ensure_ascii=False, indent=1))
