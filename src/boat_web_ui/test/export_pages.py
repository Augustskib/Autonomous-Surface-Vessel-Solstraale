"""Compose the real console page for offline JS tests, without ROS imports."""
import ast
import json
from pathlib import Path

source = Path(__file__).resolve().parents[1] / 'boat_web_ui/web_server.py'
ns = {'Path': Path, '__file__': str(source)}
tree = ast.parse(source.read_text(encoding='utf-8'))
for node in tree.body:
    if isinstance(node, ast.FunctionDef) and node.name == '_compose':
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), 'exec'), ns)
page = ns['_compose']().replace('__MANUAL_THRUST_LIMIT__', '35')
# One page serves piloting and recording; both keys are kept for the tests.
print(json.dumps({'control': page, 'data': page}))
