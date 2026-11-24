"""Render short, labelled replay clips from three CC BY tomato leaf photographs."""

import argparse
import hashlib
from pathlib import Path
import subprocess
from urllib.request import Request, urlopen

SOURCES = {
    "positive": (
        "e/eb/Late_blight_on_tomato_leaf_%287871756748%29.jpg",  # Late_blight_on_tomato_leaf_(7871756748).jpg
        "c307edf59875fa677c15ecdad3b10230d96886942fda2f743ebda8880ad2882e",
    ),
    "healthy": (
        "9/96/Healthy_tomato_leaves_%287871755330%29.jpg",  # Healthy_tomato_leaves_(7871755330).jpg
        "d18871798c866d452749fc1b54a53eff916e997ccd663622f17077fc375d480b",
    ),
    "ambiguous": (
        "7/70/Early_blight_on_tomato_leaves_%287871930010%29.jpg",  # Early_blight_on_tomato_leaves_(7871930010).jpg
        "f356736fc8203ef031bf6ea824e2c68b6d91edf6e385d92a99f5e7b3544db8",
    ),
}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    args.source_dir.mkdir(parents=True, exist_ok=True)
    args.output.mkdir(parents=True, exist_ok=True)
    for case, (remote_path, expected_hash) in SOURCES.items():
        source = args.source_dir / f"{case}.jpg"
        if not source.exists():
            request = Request(
                f"https://upload.wikimedia.org/wikipedia/commons/{remote_path}",
                headers={"User-Agent": "RingQuadReplayFixtures/1.0"},
            )
            with urlopen(request, timeout=45) as response:
                content = response.read()
            if hashlib.sha256(content).hexdigest() != expected_hash:
                raise ValueError(f"Unexpected source checksum for {case}")
            source.write_bytes(content)
        elif hashlib.sha256(source.read_bytes()).hexdigest() != expected_hash:
            raise ValueError(f"Unexpected source checksum for {source}")

        filters = (
            "scale=960:-2,zoompan=z='1.06':x='iw/2-iw/zoom/2+sin(on/4)*8':"
            "y='ih/2-ih/zoom/2+cos(on/6)*4':d=24:s=480x320:fps=8"
        )
        if case == "ambiguous":
            filters += ",gblur=sigma=7"
        subprocess.run(
            [
                "ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "error", "-n",
                "-i", str(source), "-vf", filters, "-frames:v", "24", "-codec:v",
                "libx264", "-pix_fmt", "yuv420p", str(args.output / f"{case}.mp4"),
            ],
            check=True,
        )


if __name__ == "__main__":
    main()
