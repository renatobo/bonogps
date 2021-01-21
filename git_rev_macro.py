# Ptyhon script to generate define macro statements used at compile time
# They rely on this folder being cloned from the repo, e.g. from https://github.com/renatobo/bonogps
# Author: Renato Bonomini
# More info at https://github.com/renatobo/bonogps

import subprocess
import re

revision = (
    subprocess.check_output(
        ["git", "describe", "--tags", "--always", "--dirty"])
    .strip()
    .decode("utf-8")
)

repo = (
    subprocess.check_output(["git", "config", "--get", "remote.origin.url"])
    .strip()
    .decode("utf-8")
    .replace('.git', '')
)

repolink = re.sub(r'(v[\d+\.a-zA-Z]+)-.*', r"\1", revision)

print("-DGIT_REV='\"%s\"' -DGIT_REPO='\"%s/releases/tag/%s\"'" %
      (revision, repo, repolink))
