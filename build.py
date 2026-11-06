"""Build one drag-and-drop Blender extension zip per platform, with rosbags' wheels bundled."""
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).parent
ADDON_DIR = ROOT / "blender_rosbag_importer"
DIST_DIR = ROOT / "dist"
BLENDER_PYTHON = "3.11"  # Blender 4.2 - 5.x all ship 3.11
# blender platform name -> pip platform tags to download wheels for
PLATFORMS = {
    "windows-x64": ["win_amd64"],
    "linux-x64": ["manylinux2014_x86_64", "manylinux_2_28_x86_64"],
    "macos-arm64": ["macosx_11_0_arm64"],
    "macos-x64": ["macosx_10_13_x86_64"],
}
# Blender already ships numpy, and it must match Blender's own build
SKIP_WHEELS = ("numpy",)
# rosbags requires these only on older pythons; pip evaluates the markers against the
# *host* interpreter, not --python-version, so they have to be requested explicitly
EXTRA_REQUIREMENTS = ("backports.zstd", "typing_extensions")


def _download_wheels(pip_platforms: list[str], dest: Path) -> list[Path]:
    """Download rosbags + deps as wheels for one target platform."""
    platform_args = [arg for tag in pip_platforms for arg in ("--platform", tag)]
    subprocess.run(
        [sys.executable, "-m", "pip", "download", "rosbags", *EXTRA_REQUIREMENTS, "--only-binary=:all:",
         "--python-version", BLENDER_PYTHON, "--implementation", "cp", "-d", str(dest), *platform_args],
        check=True,
    )
    return sorted(w for w in dest.glob("*.whl") if not w.name.lower().startswith(SKIP_WHEELS))


def _build_one(blender_platform: str, pip_platforms: list[str]) -> Path:
    with tempfile.TemporaryDirectory() as tmp:
        stage = Path(tmp) / "addon"
        shutil.copytree(ADDON_DIR, stage, ignore=shutil.ignore_patterns("__pycache__", "vendor", "wheels"))
        wheels = _download_wheels(pip_platforms, Path(tmp) / "dl")
        (stage / "wheels").mkdir()
        for wheel in wheels:
            shutil.copy(wheel, stage / "wheels" / wheel.name)

        wheel_lines = "".join(f'  "./wheels/{w.name}",\n' for w in wheels)
        with open(stage / "blender_manifest.toml", "a") as f:
            f.write(f'platforms = ["{blender_platform}"]\nwheels = [\n{wheel_lines}]\n')

        DIST_DIR.mkdir(exist_ok=True)
        out = DIST_DIR / f"blender_rosbag_importer-{blender_platform}"
        return Path(shutil.make_archive(str(out), "zip", stage))


def main() -> None:
    for blender_platform, pip_platforms in PLATFORMS.items():
        print(f"[build] {blender_platform}")
        print(f"[build]   -> {_build_one(blender_platform, pip_platforms)}")


if __name__ == "__main__":
    main()
