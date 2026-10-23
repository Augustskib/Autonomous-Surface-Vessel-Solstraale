"""Compose the real page templates for offline JS tests, without ROS imports."""
import ast
import json
from pathlib import Path

source = Path(__file__).resolve().parents[1] / 'boat_web_ui/web_server.py'
ns = {'Path': Path, '__file__': str(source)}
tree = ast.parse(source.read_text(encoding='utf-8'))
for node in tree.body:
    if isinstance(node, ast.Assign) and isinstance(node.value, ast.Constant) and isinstance(node.value.value, str):
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), 'exec'), ns)
    if isinstance(node, ast.FunctionDef) and node.name == '_compose':
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), 'exec'), ns)
print(json.dumps({
    name: ns['_compose'](ns[template], name).replace('{{ manual_thrust_limit }}', '35')
    for name, template in [('control', 'HTML_PAGE'), ('data', 'DATA_PAGE')]
}))
