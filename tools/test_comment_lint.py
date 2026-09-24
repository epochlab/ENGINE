#!/usr/bin/env python3
"""Lexer tests for comment_lint. Run standalone; ctest registers this as comment_lint.selftest."""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from comment_lint import MAX_COLS, check, main, scan  # noqa: E402

FAILURES: list[str] = []


def expect(name: str, got: object, want: object) -> None:
    if got != want:
        FAILURES.append(f"{name}: got {got!r}, want {want!r}")


def carries(text: str) -> set[int]:
    return scan(text)[0]


def owns(text: str) -> set[int]:
    return scan(text)[1]


# A '//' inside a string literal is code. albedo_table.cpp writes exactly this when it
# generates albedo_table.inc, and a regex gate would score the generated text as its own.
expect("string // not a comment", carries('out << "// generated";\n'), set())
expect("string with escaped quote", carries('s = "a\\"// no";\n'), set())
expect("raw string // not a comment", carries('s = R"(// no)";\n'), set())
expect("raw string with delimiter", carries('s = R"x(// no)x";\n'), set())
expect("char literal quote", carries("c = '\\'';\n"), set())

# 1'000'000 is a digit separator, not the start of a char literal that would swallow the //.
expect("digit separator", carries("n = 1'000'000; // real\n"), {0})
expect("hex digit separator", carries("n = 0xFF'FF; // real\n"), {0})

expect("trailing comment counts", carries("code(); // why\n"), {0})
expect("trailing comment not owned", owns("code(); // why\n"), set())
expect("standalone owned", owns("// why\n"), {0})
expect("indented standalone owned", owns("    // why\n"), {0})

expect("block spans lines", carries("/* a\nb\nc */\n"), {0, 1, 2})
expect("inline block not owned", owns("code(); /* a\nb */\n"), set())
expect("standalone block owned", owns("/* a\nb */\n"), {0, 1})

# A backslash at end of line splices the next line into the same // comment.
expect("continued line comment", carries("// a\\\nb\nc();\n"), {0, 1})
expect("uncontinued line comment", carries("// a\nb();\n"), {0})

expect("comment after string", carries('s = "x"; // why\n'), {0})

# A backslash-newline splices a string or char literal onto the next line. Miss it and every
# comment after the splice is reported one line early.
expect("string splice keeps line count", carries('s = "a\\\nb";\n// why\n'), {2})
expect("char splice keeps line count", carries("c = 'a\\\n'; // why\n"), {1})
expect("two splices keep line count", carries('s = "a\\\nb\\\nc";\n// why\n'), {3})
expect("slash divide not comment", carries("a = b / c;\n"), set())


def check_text(text: str) -> tuple[int, int]:
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "t.cpp"
        path.write_text(text, encoding="utf-8")
        found, stats = check(path)
        return len(found), stats.comment_lines


expect("over-column flagged", check_text("// " + "x" * MAX_COLS + "\n")[0], 1)
expect("at-budget accepted", check_text("//" + "x" * (MAX_COLS - 2) + "\n")[0], 0)
expect("three lines accepted", check_text("// a\n// b\n// c\n")[0], 0)
expect("four lines flagged", check_text("// a\n// b\n// c\n// d\n")[0], 1)
expect("split runs accepted", check_text("// a\n// b\ncode();\n// c\n// d\n")[0], 0)
expect("no trailing newline", check_text("// a")[1], 1)

# The dead-gate guard: a root with no sources must fail loudly, not report a clean tree.
with tempfile.TemporaryDirectory() as empty:
    argv = sys.argv
    sys.argv = ["comment_lint", "--root", empty]
    expect("empty root exits 2", main(), 2)
    sys.argv = argv

if FAILURES:
    for failure in FAILURES:
        print(f"FAIL {failure}", file=sys.stderr)
    sys.exit(1)
print("comment_lint selftest: all checks passed")
