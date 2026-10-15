"""对照归档检查行级注释修订；不把源码行号变化当作二进制相同保证。"""

from datetime import datetime
from pathlib import Path
import hashlib
import json
import re
import zipfile

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "docs/evidence/M2/revisions/20261012/line_comments"

# 字符串/字符常量先于注释识别，防止把常量中的//或/*错误删掉。
LEXEMES = re.compile(
    r'(?P<literal>(?:u8|u|U|L)?"(?:\\.|[^"\\])*"|(?:u|U|L)?\'(?:\\.|[^\'\\])*\')'
    r'|(?P<comment>/\*.*?\*/|//[^\n]*)'
    r'|(?P<token>[A-Za-z_][A-Za-z_0-9]*'
    r'|(?:\d|\.\d)(?:[A-Za-z_0-9.]|[eEpP][+-])*'
    r'|>>=|<<=|\.\.\.|##|->|\+\+|--|<<|>>|<=|>=|==|!=|&&|\|\|'
    r'|\+=|-=|\*=|/=|%=|&=|\^=|\|=|[^\s])',
    re.DOTALL,
)


def canonical(source):
    # C翻译阶段先拼接反斜线换行，再替换注释；保留注释内换行用于预处理指令边界。
    source = source.replace("\r\n", "\n").replace("\\\n", "")
    tokens = []
    cleaned = []
    end = 0
    for match in LEXEMES.finditer(source):
        cleaned.append(source[end:match.start()])
        if match.lastgroup == "comment":
            cleaned.append(" " + "\n" * match.group().count("\n"))
        else:
            tokens.append(match.group())
            cleaned.append(match.group())
        end = match.end()
    cleaned.append(source[end:])
    directives = []
    for line in "".join(cleaned).splitlines():
        if line.lstrip().startswith("#"):
            directives.append(tuple(m.group() for m in LEXEMES.finditer(line)))
    return tokens, directives


def main():
    before = json.loads((EVIDENCE / "before_manifest.json").read_text(encoding="utf-8"))
    checks = []
    with zipfile.ZipFile(EVIDENCE / "before_firmware_sources.zip") as archive:
        for entry in before:
            old = archive.read(entry["archive_name"].replace("\\", "/"))
            path = ROOT / entry["path"]
            new = path.read_bytes()
            encoding = "utf-8"
            try:
                old_text = old.decode(encoding)
            except UnicodeDecodeError:
                encoding = "gbk"
                old_text = old.decode(encoding)
            new_text = new.decode(encoding)
            old_tokens, old_directives = canonical(old_text)
            new_tokens, new_directives = canonical(new_text)
            leading_license = old_text.startswith("/*")
            license_same = not leading_license or old_text.split("*/", 1)[0] == new_text.split("*/", 1)[0]
            check = {
                "path": entry["path"], "encoding": encoding,
                "before_sha256": hashlib.sha256(old).hexdigest(),
                "after_sha256": hashlib.sha256(new).hexdigest(),
                "changed": old != new, "token_count": len(old_tokens),
                "code_tokens_equal": old_tokens == new_tokens,
                "preprocessor_directives_equal": old_directives == new_directives,
                "license_preserved": license_same,
                "bom_preserved": old.startswith(b"\xef\xbb\xbf") == new.startswith(b"\xef\xbb\xbf"),
                "before_crlf": old.count(b"\r\n"), "after_crlf": new.count(b"\r\n"),
                "before_bare_lf": old.count(b"\n") - old.count(b"\r\n"),
                "after_bare_lf": new.count(b"\n") - new.count(b"\r\n"),
            }
            check["pass"] = all(check[key] for key in (
                "code_tokens_equal", "preprocessor_directives_equal", "license_preserved", "bom_preserved"))
            check["pass"] &= (check["before_bare_lf"] == 0) == (check["after_bare_lf"] == 0)
            check["pass"] &= check["before_sha256"] == entry["sha256"]
            checks.append(check)
    report = {"generated_at": datetime.now().astimezone().isoformat(),
              "baseline_archive": "before_firmware_sources.zip", "files": checks,
              "changed_files": sum(item["changed"] for item in checks),
              "all_pass": all(item["pass"] for item in checks),
              "scope": "C token/directive identity, encoding/newline/license preservation",
              "limitations": "__LINE__/debug line tables and diagnostics can change; no hardware evidence"}
    (EVIDENCE / "comment_only_check.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Comment audit: {len(checks)} files, {report['changed_files']} changed, all_pass={report['all_pass']}")
    if not report["all_pass"]:
        for check in checks:
            if not check["pass"]:
                print(check["path"], "FAILED")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
