"""Read-only inspection of project resources; never imports model or robot code."""
import ast
import collections
import csv
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'records/inventory'

def inspect_code(source):
    try:
        tree = ast.parse(source)
    except SyntaxError as exc:
        return {'syntax': str(exc)}
    imports, functions, calls = set(), [], set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.update(a.name for a in node.names)
        elif isinstance(node, ast.ImportFrom):
            imports.add(node.module or '')
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            functions.append({'name': node.name, 'line': node.lineno, 'kind': type(node).__name__})
        elif isinstance(node, ast.Call):
            calls.add(ast.unparse(node.func))
    return {'syntax': 'parsed-not-executed', 'imports': sorted(imports), 'symbols': functions, 'calls': sorted(calls)}

def category(path):
    p = path.as_posix()
    if p.startswith('scripts/'):
        return '资料维护脚本'
    if '/采集缓存/' in p or '_files/' in p:
        return '网页采集依赖-非机器人应用'
    if path.suffix in ('.py', '.ipynb', '.h', '.so'):
        return '代码或接口库'
    if path.suffix in ('.om', '.onnx', '.pth') or path.name.endswith('.pth.tar'):
        return '模型'
    if path.suffix.lower() == '.stl':
        return '机械结构网格'
    if path.suffix in ('.exe', '.plugin'):
        return '安装工具'
    if path.suffix in ('.zip', '.gz'):
        return '原始压缩资料'
    if path.suffix in ('.mp4', '.wav'):
        return '演示或音频样本'
    if path.suffix in ('.jpg', '.png', '.jpeg', '.gif', '.svg'):
        return '图片样本或文档配图'
    return '文档或辅助资源'

def main():
    OUT.mkdir(parents=True, exist_ok=True)
    rows, code = [], []
    for path in sorted(ROOT.rglob('*')):
        relative = path.relative_to(ROOT)
        if not path.is_file() or any(part in ('.codegraph', '.git', '__pycache__', 'docs') for part in relative.parts):
            continue
        row = {'path': relative.as_posix(), 'category': category(relative), 'bytes': path.stat().st_size}
        if path.suffix in ('.py', '.ipynb', '.h', '.so', '.om', '.onnx', '.pth') or path.name.endswith('.pth.tar'):
            digest = hashlib.sha256()
            with path.open('rb') as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                    digest.update(chunk)
            row['sha256'] = digest.hexdigest()
        rows.append(row)
        if path.suffix == '.py':
            code.append({'path': row['path'], **inspect_code(path.read_text(encoding='utf-8-sig'))})
        elif path.suffix == '.ipynb':
            notebook = json.loads(path.read_text(encoding='utf-8-sig'))
            cells = []
            for index, cell in enumerate(notebook.get('cells', [])):
                if cell['cell_type'] == 'code':
                    cells.append({'cell_index_zero_based': index, **inspect_code(''.join(cell.get('source', [])))})
            code.append({'path': row['path'], 'cells': cells})
    with (OUT / '全部资源.csv').open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=['path', 'category', 'bytes', 'sha256'])
        writer.writeheader()
        writer.writerows(rows)
    (OUT / '代码结构.json').write_text(json.dumps(code, ensure_ascii=False, indent=2), encoding='utf-8')
    counts = dict(collections.Counter(row['category'] for row in rows))
    (OUT / '盘点统计.json').write_text(json.dumps({'scope': '工作区文件，不展开工具环境tar包；排除docs和索引缓存；不等于物理硬件实测', 'files': len(rows), 'categories': counts, 'python': sum(Path(r['path']).suffix == '.py' for r in rows), 'notebooks': sum(Path(r['path']).suffix == '.ipynb' for r in rows)}, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({'files': len(rows), 'code_entries': len(code), 'categories': counts}, ensure_ascii=False))

if __name__ == '__main__':
    main()
