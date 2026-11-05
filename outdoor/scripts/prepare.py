"""Install versioned outdoor sources; never start a driver or send a command."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess

SOURCE=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination",type=Path)
    args=parser.parse_args()
    target=args.destination.expanduser().resolve()
    if target==SOURCE:
        raise ValueError("Choose a deployment directory outside this source checkout")
    for name in ("robot_outdoor","scripts","tests","config","capabilities"):
        shutil.copytree(SOURCE/name,target/name,dirs_exist_ok=True,
                        ignore=shutil.ignore_patterns("__pycache__","secrets.json","gnss.base.yaml"))
    for name in ("outdoor","package_manifest.yaml","sources.json","README.md"):
        shutil.copy2(SOURCE/name,target/name)
    for name in ("check.yaml","field.yaml","scene_navigation.json"):
        file=target/"config"/name
        file.write_text(file.read_text().replace("/home/nvidia/robot-outdoor-rbnx",str(target)))
    for item in json.loads((SOURCE/"sources.json").read_text()):
        folder=target/item["path"]
        if not folder.exists():
            folder.parent.mkdir(parents=True,exist_ok=True)
            subprocess.run(["git","clone","--filter=blob:none",item["url"],str(folder)],check=True)
        if not (folder/".git").exists():
            raise ValueError("Existing package has no Git history: "+str(folder))
        dirty=subprocess.check_output(["git","-C",str(folder),"status","--porcelain"],text=True)
        if dirty:
            raise ValueError("Preserve local package changes before updating: "+str(folder))
        subprocess.run(["git","-C",str(folder),"fetch","origin",item["revision"]],check=True)
        subprocess.run(["git","-C",str(folder),"checkout","--detach",item["revision"]],check=True)
    config=target/".robonix"/"config.yaml"
    config.parent.mkdir(parents=True,exist_ok=True)
    config.write_text(json.dumps({"package_storage_path":str(target/".robonix/packages"),
                                 "robonix_source_path":str(target/"framework/robonix")},indent=2))
    base=target/"config/gnss.base.yaml"
    if not base.exists():
        shutil.copy2(target/"config/gnss.base.example.yaml",base)
        base.chmod(0o600)
    print("Sources prepared. Configure GNSS credentials and ROS paths, then follow README.")

if __name__=="__main__":main()
