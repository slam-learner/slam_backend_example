import os
import json
from pathlib import Path

def find_compile_commands(root_dir: Path):
    """递归查找所有 compile_commands.json 文件"""
    return list(root_dir.rglob("compile_commands.json"))

def load_commands(file_path: Path):
    """读取单个 compile_commands.json"""
    try:
        with open(file_path, 'r') as f:
            data = json.load(f)
        return data
    except Exception as e:
        print(f"Failed to load {file_path}: {e}")
        return []

def merge_commands(all_commands):
    """合并所有命令，去重（可选）"""
    unique = {}
    for cmd_list in all_commands:
        for entry in cmd_list:
            key = (entry["file"], entry["directory"])
            # 最后一个覆盖同 key 的
            unique[key] = entry
    return list(unique.values())

def main():
    project_root = Path.cwd()
    all_cc_files = find_compile_commands(project_root)

    print(f"Found {len(all_cc_files)} compile_commands.json files.")
    all_commands = []
    for cc_file in all_cc_files:
        cmds = load_commands(cc_file)
        all_commands.append(cmds)

    merged = merge_commands(all_commands)

    output_path = project_root / "compile_commands.combined.json"
    with open(output_path, 'w') as f:
        json.dump(merged, f, indent=2)

    print(f"Merged {sum(len(c) for c in all_commands)} entries "
          f"into {len(merged)} unique entries.")
    print(f"Output written to: {output_path}")

    # 可选：生成一个软链接为 clangd 默认使用名
    symlink_path = project_root / "compile_commands.json"
    if not symlink_path.exists():
        symlink_path.symlink_to(output_path.name)
        print(f"Symlink created: {symlink_path} → {output_path.name}")

if __name__ == "__main__":
    main()
