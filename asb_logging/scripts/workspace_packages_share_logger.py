#! /usr/bin/python3

import os
import shutil
from pathlib import Path

import rclpy
from rclpy.node import Node

# Patterns to ignore (full glob patterns relative to share/{pkg}/)
IGNORE_PATTERNS = [
    "cmake",             # directory
    "environment",       # directory
    "hook",              # directory
    "local_setup.*",     # files
    "package.*",         # files
]


def should_ignore(path: Path) -> bool:
    """Return True if the file or directory should be ignored."""
    for pattern in IGNORE_PATTERNS:
        if path.match(pattern):
            return True
    return False


def copy_follow_symlink(src: Path, dst: Path, verbose: bool) -> None:
    """Copy file or directory, resolving symlinks to their targets."""
    if src.is_symlink():
        target = src.resolve()
        if target.is_dir():
            if verbose:
                print(
                    f"shutil.copytree\n"
                    f"    target={target}\n"
                    f"    dst={dst}\n"
                    f"    symlinks=False)"
                )
            shutil.copytree(target, dst, symlinks=False)
        else:
            if verbose:
                print(
                    f"shutil.copy2\n"
                    f"    target={target}\n"
                    f"    dst={dst}"
                )
            shutil.copy2(target, dst)
    elif src.is_dir():
        if verbose:
            print(
                f"shutil.copytree\n"
                f"    src={src}\n"
                f"    dst={dst}\n"
                f"    symlinks=False"
            )
        shutil.copytree(src, dst, symlinks=False)
    else:
        if verbose:
            print(
                f"shutil.copy2\n"
                f"    src={src}\n"
                f"    dst={dst}"
            )
        shutil.copy2(src, dst)


def log_workspace_packages_share(workspace_install_dir: str, output_dir: str, verbose: bool = False) -> None:
    # Iterate over each package directory in the install directory
    for pkg_dir in Path(workspace_install_dir).iterdir():
        share_path = pkg_dir / "share" / pkg_dir.name
        if not share_path.exists():
            continue
        if verbose:
            print(f"\n * share_path: {share_path}")

        dest_pkg_dir = Path(output_dir) / pkg_dir.name
        dest_pkg_dir.mkdir(parents=True, exist_ok=True)
        if verbose:
            print(f"   dest_pkg_dir: {dest_pkg_dir}")

        for item in share_path.iterdir():
            if should_ignore(item.relative_to(share_path)):
                continue

            src_path = item
            dst_path = dest_pkg_dir / item.name

            try:
                copy_follow_symlink(src_path, dst_path, verbose)
            except Exception as e:
                print(f"Failed to copy {src_path} -> {dst_path}: {e}")


class WorkspacePackagesShareLogger(Node):
    def __init__(self):
        super().__init__('workspace_packages_share_logger')

        self.declare_parameter('workspace_install_dir_path', rclpy.Parameter.Type.STRING)
        workspace_install_dir_path = os.path.expanduser(self.get_parameter('workspace_install_dir_path').get_parameter_value().string_value)

        self.declare_parameter('log_dir_path', rclpy.Parameter.Type.STRING)
        log_dir_path = os.path.join(os.path.expanduser(self.get_parameter('log_dir_path').get_parameter_value().string_value), "workspace_packages_share")

        log_workspace_packages_share(workspace_install_dir=workspace_install_dir_path, output_dir=log_dir_path)


def main(args=None):
    rclpy.init(args=args)
    node = WorkspacePackagesShareLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
