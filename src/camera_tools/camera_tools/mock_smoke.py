"""运行无硬件 Mock 演示并检查点查询结果。"""

import argparse
import subprocess
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description="运行相机 Mock 冒烟测试")
    parser.add_argument("binary", help="camera_manager_demo 可执行文件")
    args = parser.parse_args()
    result = subprocess.run([args.binary], check=False, text=True, capture_output=True)
    sys.stdout.write(result.stdout)
    sys.stderr.write(result.stderr)
    if result.returncode != 0 or "point_code=OK" not in result.stdout:
        print("Mock 冒烟测试失败", file=sys.stderr)
        return 1
    print("Mock 冒烟测试通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
