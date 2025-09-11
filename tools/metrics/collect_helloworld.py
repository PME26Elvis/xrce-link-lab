#!/usr/bin/env python3
import argparse, json, os, re, sys
from datetime import datetime

def read_text(path):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            txt = f.read()
        return txt, True
    except Exception:
        return "", False

def summarize_text(txt, maxlen=200):
    if not txt:
        return ""
    t = txt.strip().splitlines()
    head = "\n".join(t[:5])
    return head[:maxlen]

def main():
    ap = argparse.ArgumentParser(description="Collect minimal metrics for XRCE HelloWorld jobs.")
    ap.add_argument("--agent-log", required=True)
    ap.add_argument("--pub-out", required=True)
    ap.add_argument("--sub-out", required=True)
    ap.add_argument("--netem", default=None, help="Optional path to netem.txt")
    ap.add_argument("--label", default="baseline")
    ap.add_argument("--out", default="metrics.json")
    args = ap.parse_args()

    agent_txt, agent_ok = read_text(args.agent_log)
    pub_txt, pub_ok = read_text(args.pub_out)
    sub_txt, sub_ok = read_text(args.sub_out)
    netem_txt, netem_ok = ("", False)
    if args.netem:
        netem_txt, netem_ok = read_text(args.netem)

    def count_lines(s): return 0 if not s else len(s.splitlines())
    def size_bytes(p): 
        try: return os.path.getsize(p)
        except Exception: return 0

    # 不臆測完整文案；僅偵測「2019 埠/udp4」等關鍵片語是否存在
    agent_has_udp2019 = ("2019" in agent_txt) or ("udp4" in agent_txt.lower())
    # 非空即視為有輸出
    pub_nonempty = bool(pub_txt.strip())
    sub_nonempty = bool(sub_txt.strip())

    metrics = {
        "label": args.label,
        "timestamp_utc": datetime.utcnow().isoformat() + "Z",
        "files": {
            "agent_log": {"exists": agent_ok, "bytes": size_bytes(args.agent_log), "lines": count_lines(agent_txt)},
            "pub_out":   {"exists": pub_ok,   "bytes": size_bytes(args.pub_out),   "lines": count_lines(pub_txt), "nonempty": pub_nonempty},
            "sub_out":   {"exists": sub_ok,   "bytes": size_bytes(args.sub_out),   "lines": count_lines(sub_txt), "nonempty": sub_nonempty},
        },
        "agent_log_contains_udp2019_or_udp4": agent_has_udp2019,
        "netem": {
            "provided": netem_ok,
            "summary_head": summarize_text(netem_txt, 200) if netem_ok else ""
        }
    }

    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2)
    print(f"Wrote {args.out}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
