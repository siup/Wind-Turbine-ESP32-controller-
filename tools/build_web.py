# PlatformIO pre-script: prepares the LittleFS image content for the web panel.
#
#   data/<file>.html|js|css|json|svg|webmanifest -> .pio/webfs/<file>.gz (gzip -9, mtime 0)
#   any other file (png, ...)                     -> .pio/webfs/<file>    (copied as is)
#
# Only the .gz goes into the image: the web server sends it with
# Content-Encoding: gzip when /<file> is requested. PROJECT_DATA_DIR is then
# pointed at .pio/webfs, so `pio run -t buildfs / uploadfs` packs that folder.
#
# index.html is slimmed first: newlines inside <style> go, and inside the
# inline <script> leading indentation and whole-line // comments are dropped
# and a line ending in ; { or , is joined with the next one (no // on it).
# None of that changes what the code does. The source stays readable; the
# budget applies to what ships.
#
# Runs before every build, so it is incremental: a file is rewritten only when
# its content changes (deterministic gzip), stale outputs are removed. The
# build fails if the slimmed index.html or data/viz.js is over its budget.
import gzip
import os

Import("env")  # noqa: F821  (provided by PlatformIO / SCons)

INDEX_LIMIT = 61440  # bytes, slimmed index.html before gzip (spec: 60 kB)
VIZ_LIMIT = 24576    # bytes, data/viz.js (3D view, loaded on demand) before gzip
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


def slim_html(data):
    """Slim the inline <style> and <script> blocks of the panel.

    Lines inside a multi-line template literal are left alone (odd number of
    backticks seen so far), so their text never changes. Joining a line that
    ends in ; { or , with the next is safe: no semicolon insertion happens
    there. A line with // anywhere (URL in a string, trailing comment) is
    never joined.
    """
    text = data.decode("utf-8")
    out = []
    mode = ""  # "", "style" or "script"
    in_tpl = False
    glue = False  # previous script line may take the next one on the same line
    for line in text.split("\n"):
        low = line.lower()
        if not mode:
            out.append(line)
            if "<style" in low and "</style>" not in low:
                mode = "style"
            elif "<script" in low and "</script>" not in low and "src=" not in low:
                mode = "script"
            glue = False
            continue
        if "</" + mode + ">" in low:
            mode = ""
            out.append(line)
            continue
        if mode == "style":
            s = line.strip()
            if glue and s:
                out[-1] += ("" if out[-1][-1] in "{};," else " ") + s
            elif s:
                out.append(s)
                glue = True
            continue
        if in_tpl:
            out.append(line)
            glue = False
        else:
            s = line.lstrip(" \t")
            if s.startswith("//") or not s:
                continue
            if glue:
                out[-1] += s
            else:
                out.append(s)
            glue = s[-1] in ";{," and "//" not in s and s.count("`") % 2 == 0
        if line.count("`") % 2:
            in_tpl = not in_tpl
    return "\n".join(out).encode("utf-8")


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

    for name, limit, slim in (("index.html", INDEX_LIMIT, True), ("viz.js", VIZ_LIMIT, False)):
        path = os.path.join(src, name)
        if not os.path.isfile(path):
            continue
        with open(path, "rb") as f:
            data = f.read()
        size = len(slim_html(data)) if slim else len(data)
        if size > limit:
            print("")
            print("build_web: ERROR data/%s is %d bytes%s, the limit is %d bytes."
                  % (name, size, " after slimming" if slim else "", limit))
            print("build_web: the panel must stay small (served gzipped from LittleFS).")
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
            if rel == "index.html":
                data = slim_html(data)
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
