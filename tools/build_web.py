# PlatformIO pre-script: prepares the LittleFS image content for the web panel.
#
#   data/<file>.html|js|css|json|svg|webmanifest -> .pio/webfs/<file>.gz (gzip -9, mtime 0)
#   any other file (png, ...)                     -> .pio/webfs/<file>    (copied as is)
#
# Only the .gz goes into the image: the web server sends it with
# Content-Encoding: gzip when /<file> is requested. PROJECT_DATA_DIR is then
# pointed at .pio/webfs, so `pio run -t buildfs / uploadfs` packs that folder.
#
# Runs before every build, so it is incremental: a file is rewritten only when
# its content changes (deterministic gzip), stale outputs are removed. The
# build fails if data/index.html is larger than the panel budget.
import gzip
import os

Import("env")  # noqa: F821  (provided by PlatformIO / SCons)

INDEX_LIMIT = 61440  # bytes, data/index.html before gzip (spec: 60 kB)
GZIP_EXT = (".html", ".htm", ".js", ".css", ".json", ".svg", ".webmanifest", ".txt", ".map")


def _write_if_changed(path, data):
    try:
        with open(path, "rb") as f:
            if f.read() == data:
                return False
    except OSError:
        pass
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(data)
    os.replace(tmp, path)
    return True


def _gzip(data):
    # mtime=0 and no file name in the header: same input, same bytes.
    return gzip.compress(data, compresslevel=9, mtime=0)


def build_web(env):
    project = env.subst("$PROJECT_DIR")
    src = os.path.join(project, "data")
    out = os.path.join(project, ".pio", "webfs")
    if not os.path.isdir(src):
        print("build_web: no data/ folder, web panel skipped")
        return

    index = os.path.join(src, "index.html")
    if os.path.isfile(index):
        size = os.path.getsize(index)
        if size > INDEX_LIMIT:
            print("")
            print("build_web: ERROR data/index.html is %d bytes, the limit is %d bytes." % (size, INDEX_LIMIT))
            print("build_web: the panel must stay small (one file, served gzipped from LittleFS).")
            print("")
            env.Exit(1)

    os.makedirs(out, exist_ok=True)
    wanted = set()
    rows = []
    changed = 0
    for root, dirs, files in os.walk(src):
        dirs[:] = sorted(d for d in dirs if not d.startswith("."))
        for name in sorted(files):
            if name.startswith("."):
                continue
            path = os.path.join(root, name)
            rel = os.path.relpath(path, src)
            with open(path, "rb") as f:
                data = f.read()
            if name.lower().endswith(GZIP_EXT):
                rel_out = rel + ".gz"
                blob = _gzip(data)
            else:
                rel_out = rel
                blob = data
            dst = os.path.join(out, rel_out)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            if _write_if_changed(dst, blob):
                changed += 1
            wanted.add(os.path.normpath(rel_out))
            rows.append((rel_out.replace(os.sep, "/"), len(data), len(blob)))

    # Remove outputs whose source is gone (renamed or deleted files).
    for root, dirs, files in os.walk(out, topdown=False):
        for name in files:
            p = os.path.join(root, name)
            if os.path.normpath(os.path.relpath(p, out)) not in wanted:
                os.remove(p)
                changed += 1
        if root != out and not os.listdir(root):
            os.rmdir(root)

    total = sum(r[2] for r in rows)
    print("build_web: data/ -> .pio/webfs (%d changed)" % changed)
    print("  %-24s %9s %9s" % ("file", "source", "in image"))
    for name, raw, packed in rows:
        print("  %-24s %9d %9d" % (name, raw, packed))
    print("  %-24s %9s %9d" % ("total", "", total))
    env.Replace(PROJECT_DATA_DIR=out)


# Host unit tests (env:native) have no filesystem image.
if env.get("PIOENV") != "native" and env.get("PIOPLATFORM") != "native":  # noqa: F821
    build_web(env)  # noqa: F821
