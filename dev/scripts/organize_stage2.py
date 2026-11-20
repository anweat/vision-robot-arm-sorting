"""Organize the authenticated course export; no cookies are stored or needed here."""
import concurrent.futures
import hashlib
import html
import json
import os
import pathlib
import re
import urllib.parse
import urllib.request

from bs4 import BeautifulSoup

ROOT = pathlib.Path(__file__).resolve().parents[2]
OUT = ROOT / '资料/课程说明/第二阶段-机械臂基础控制'
BASE = 'http://10.126.59.136:30080'
SOURCE = BASE + '/course/previewCourseProject/c17d2b20c3b84c898d74796a25be9c02/1'
DATA = json.loads((OUT / '课程原始内容.json').read_text(encoding='utf-8'))
(OUT / 'assets').mkdir(exist_ok=True)
(OUT / '代码片段').mkdir(exist_ok=True)
urls = set()
for task in DATA:
    for key in ('introduction', 'step', 'solution'):
        soup = BeautifulSoup(task[key], 'html.parser')
        for img in soup.find_all('img', src=True):
            urls.add(urllib.parse.urljoin(BASE, img['src']))

def download(url):
    if urllib.parse.urlparse(url).hostname not in ('10.126.59.136', 'tecs-prod-static.obs.cn-north-1.myhuaweicloud.com', 'www.hiascend.com'):
        return {'url': url, 'status': 'external-skipped'}
    suffix = pathlib.Path(urllib.parse.unquote(urllib.parse.urlparse(url).path)).suffix
    name = hashlib.sha256(url.encode()).hexdigest()[:20] + (suffix if suffix in ('.png', '.jpg', '.jpeg', '.gif', '.svg') else '.img')
    target = OUT / 'assets' / name
    try:
        if not target.exists():
            with urllib.request.urlopen(url, timeout=25) as response:
                content_type = response.headers.get_content_type()
                if not content_type.startswith('image/'):
                    raise ValueError('Expected image, got ' + content_type)
                target.write_bytes(response.read())
        return {'url': url, 'status': 'downloaded', 'local': 'assets/' + name, 'bytes': target.stat().st_size}
    except Exception as exc:
        return {'url': url, 'status': 'failed', 'error': str(exc)}

with concurrent.futures.ThreadPoolExecutor(max_workers=6) as executor:
    resources = list(executor.map(download, sorted(urls)))
mapping = {r['url']: r['local'] for r in resources if r['status'] == 'downloaded'}
video_root = ROOT / '资料/原始解压/软件开发与实训综合实践资料分享/基于计算机视觉的智能机械臂应用实践/视频'
for task in DATA:
    for key in ('introduction', 'step', 'solution'):
        soup = BeautifulSoup(task[key], 'html.parser')
        for tag in soup.find_all('iframe', title=True):
            local = video_root / tag['title']
            if local.is_file():
                for attr in ('src', 'href'):
                    if tag.get(attr):
                        mapping[urllib.parse.urljoin(BASE, tag[attr])] = pathlib.Path(os.path.relpath(local, OUT)).as_posix()
        for tag in soup.find_all('video', src=True):
            url = tag['src']
            if urllib.parse.urlparse(url).hostname == 'staticfile.eduplus.net':
                target = OUT / 'assets' / (hashlib.sha256(url.encode()).hexdigest()[:20] + '.mp4')
                try:
                    if not target.exists():
                        with urllib.request.urlopen(url, timeout=30) as response:
                            if response.headers.get_content_type() not in ('video/mp4', 'application/octet-stream'):
                                raise ValueError('Unexpected video content type')
                            target.write_bytes(response.read())
                    mapping[url] = 'assets/' + target.name
                    resources.append({'url': url, 'status': 'downloaded', 'local': mapping[url], 'bytes': target.stat().st_size})
                except Exception as exc:
                    resources.append({'url': url, 'status': 'failed', 'error': str(exc)})
(OUT / '资源下载清单.json').write_text(json.dumps(resources, ensure_ascii=False, indent=2), encoding='utf-8')
style = 'body{font:16px/1.8 system-ui,sans-serif;max-width:1000px;margin:32px auto;padding:0 24px;color:#202124}img{max-width:100%;height:auto}table{border-collapse:collapse;max-width:100%;display:block;overflow:auto}td,th{border:1px solid #ccc;padding:8px}pre{white-space:pre-wrap;background:#f5f6f7;padding:16px;overflow:auto}a{color:#1558b0}.note{background:#fff6de;padding:12px}figure{margin:20px 0}iframe{width:100%;height:400px}'
index = ['# 第二阶段：机械臂基础控制', '', '来源：' + SOURCE, '', '保存当前学生可见的任务1–7。HTML含已下载配图和课程公开代码；示例未连接硬件运行。', '', '|任务|说明|代码块|', '|---|---|---|']
for number, task in enumerate(DATA, 1):
    stem = f'{number:02d}-' + re.sub(r'[\\/:*?"<>|]', '-', task['name'])
    sections, text_parts, code_parts = [], [], []
    for key, title in [('introduction', '任务描述'), ('step', '任务指导'), ('solution', '任务实现')]:
        soup = BeautifulSoup(task[key], 'html.parser')
        for unwanted in soup.find_all(['script', 'object', 'embed']):
            unwanted.decompose()
        for tag in soup.find_all(True):
            for attr in list(tag.attrs):
                if attr.lower().startswith('on'):
                    del tag[attr]
            for attr in ('src', 'href'):
                if tag.has_attr(attr):
                    raw = str(tag[attr])
                    if raw.lower().startswith('javascript:'):
                        del tag[attr]
                    else:
                        absolute = urllib.parse.urljoin(BASE, raw)
                        tag[attr] = mapping.get(absolute, absolute)
        for iframe in soup.find_all('iframe'):
            link = soup.new_tag('a', href=iframe.get('src', SOURCE))
            link.string = iframe.get('title', '课程嵌入资源') + ('（本地视频）' if not iframe.get('src', '').startswith('http') else '（在线预览）')
            iframe.replace_with(link)
        for block_number, block in enumerate(soup.find_all('pre'), 1):
            code = block.get_text().replace('\xa0', ' ')
            code_parts.append(f'## {title} · 代码块{block_number}\n\n```\n{code}\n```\n')
            # Keep independent blocks independent, preserving the course's execution order.
            if re.search(r'(?m)^\s*(?:import |from |def |class )', code):
                code_path = OUT / '代码片段' / f'{number:02d}-{key}-{block_number:02d}.py'
                code_path.write_text('# 课程原始代码片段；须结合实验说明按顺序执行，未进行硬件验证。\n' + code + '\n', encoding='utf-8')
        sections.append('<h2>' + title + '</h2>' + (str(soup) if task[key] else '<p>该字段为空。</p>'))
        text_parts.append(title + '\n\n' + soup.get_text('\n'))
    doc = '<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>' + html.escape(task['name']) + '</title><style>' + style + '</style></head><body><a href="index.html">返回目录</a><h1>' + html.escape(task['name']) + '</h1><p>课程建议工时：' + str(task['taskHour']) + ' 小时</p><p class="note">课程资料离线归档。代码按原文保留，尚未进行设备运行验证。图片已本地化，视频链接复用已有本地文件。</p>' + ''.join(sections) + '</body></html>'
    (OUT / (stem + '.html')).write_text(doc, encoding='utf-8-sig')
    (OUT / (stem + '.txt')).write_text(task['name'] + '\n\n' + '\n\n'.join(text_parts), encoding='utf-8-sig')
    (OUT / '代码片段' / (stem + '.md')).write_text('# ' + task['name'] + '\n\n代码块保持原文及顺序，包含环境命令与Python片段，不能整体作为一个Python文件运行。\n\n' + '\n'.join(code_parts), encoding='utf-8-sig')
    index.append(f'|{task["name"]}|[图文说明]({stem}.html)|[原始代码块](代码片段/{stem}.md)|')
index += ['', f'配图下载：{sum(r["status"] == "downloaded" for r in resources if r["url"] in urls)}/{len(urls)}。详见资源下载清单。', '', '两项嵌入视频链接复用原始解压目录中的RGB灯和机械臂跳舞视频；额外的RGB演示视频下载到assets目录。', '', '当前接口只返回任务1–7；阶段总工时显示13小时，已发布任务工时合计8小时。原始包中任务8–10仍为空目录，不能视为已发布或已下载。']
(OUT / 'README.md').write_text('\n'.join(index) + '\n', encoding='utf-8-sig')
links = ''.join('<li><a href="' + html.escape(f'{i:02d}-' + re.sub(r'[\\/:*?"<>|]', '-', t['name']) + '.html', quote=True) + '">' + html.escape(t['name']) + '</a></li>' for i, t in enumerate(DATA, 1))
(OUT / 'index.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>第二阶段资料目录</title><style>' + style + '</style><h1>第二阶段：机械臂基础控制</h1><p>当前已发布的7项任务，包含说明、配图与公开代码。</p><ol>' + links + '</ol><p>示例尚未连接硬件运行。视频链接复用已有本地文件。</p></html>', encoding='utf-8-sig')
print(json.dumps({'tasks': len(DATA), 'images': sum(r['status'] == 'downloaded' for r in resources if r['url'] in urls), 'image_total': len(urls), 'failures': [r for r in resources if r['status'] != 'downloaded']}, ensure_ascii=False))
