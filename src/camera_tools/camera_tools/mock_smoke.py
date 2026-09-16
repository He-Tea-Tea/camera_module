"""运行无硬件Mock演示并检查三维查询与会话UUID重建语义。"""

from __future__ import annotations

import argparse
import subprocess
import sys


def main() -> int:
    """执行camera_manager_demo，输出原始日志并检查v0.2.4关键标志。"""
    parser = argparse.ArgumentParser(description="运行camera_module Mock冒烟测试")
    parser.add_argument("binary", help="camera_manager_demo可执行文件")
    args = parser.parse_args()

    # capture_output让工具既能做机器判定，也能把原始日志完整回显给使用者。
    result = subprocess.run(
        [args.binary],
        check=False,
        text=True,
        capture_output=True,
    )
    sys.stdout.write(result.stdout)
    sys.stderr.write(result.stderr)

    # v0.2.4至少要求三维点查询成功，并且stop/start后streaming session UUID发生变化。
    required_markers = ("point_code=OK", "session_changed=true")
    if result.returncode != 0 or any(marker not in result.stdout for marker in required_markers):
        print("Mock冒烟测试失败", file=sys.stderr)
        return 1

    print("Mock冒烟测试通过：三维查询正常，会话UUID重建正常")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
