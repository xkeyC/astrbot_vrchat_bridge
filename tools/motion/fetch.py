"""Downloads the sources of the motion library (library.CLIPS) into DATA_DIR.

    python fetch.py DATA_DIR

- CMU: the BVH conversion at github.com/una-dinosauria/cmu-mocap (one
  directory per subject, three digits);
- Bandai Namco Research Motion Dataset 1 (CC BY-NC 4.0);
- Quaternius Universal Animation Library 1 and 2 (CC0) are not on a plain
  URL: download "Universal Animation Library" and "Universal Animation
  Library 2" (Standard) from quaternius.com and copy each zip's
  Godot/*.glb to DATA_DIR/ual1.glb and DATA_DIR/ual2.glb.
"""

import os
import sys
import urllib.request

from library import CLIPS

CMU = "https://raw.githubusercontent.com/una-dinosauria/cmu-mocap/master/data/{subject:03d}/{trial}.bvh"
BANDAI = ("https://raw.githubusercontent.com/BandaiNamcoResearchInc/Bandai-Namco-Research-Motiondataset/"
          "master/dataset/Bandai-Namco-Research-Motiondataset-1/data/{name}")


def url(kind, source):
    name = os.path.basename(source)
    if kind == "cmu":
        trial = name[:-len(".bvh")]
        return CMU.format(subject=int(trial.split("_")[0]), trial=trial)
    if kind == "bandai":
        return BANDAI.format(name=name)
    return None


def main():
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    data = sys.argv[1]
    manual = set()
    for kind, source, _ in CLIPS.values():
        if kind == "gltf":
            manual.add(source)
            continue
        link = url(kind, source)
        if link is None:
            continue  # generated (keys, postures)
        path = os.path.join(data, source)
        if os.path.exists(path):
            continue
        os.makedirs(os.path.dirname(path), exist_ok=True)
        print(f"{source} <- {link}")
        urllib.request.urlretrieve(link, path)
    for source in sorted(manual):
        if not os.path.exists(os.path.join(data, source)):
            print(f"{source}: download by hand (see the top of this file)")


if __name__ == "__main__":
    main()
