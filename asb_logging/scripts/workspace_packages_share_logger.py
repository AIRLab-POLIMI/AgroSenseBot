#! /usr/bin/python3

import os
import shutil
import glob
from datetime import datetime
from pathlib import Path

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


def log_workspace_packages_share(root_dir: str, output_dir: str, verbose: bool = False) -> None:
    # Iterate over each package directory in the install directory
    for pkg_dir in Path(root_dir).iterdir():
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


if __name__ == '__main__':
    filename_stamp = datetime.now().strftime("%Y-%m-%d__%H-%M-%S")

    test_root_dir = os.path.expanduser("~/w/agrosensebot_ws/install")
    test_output_dir = os.path.expanduser(f"~/tmp/log_workspace_packages_share/{filename_stamp}/")
    log_workspace_packages_share(root_dir=test_root_dir, output_dir=test_output_dir)
