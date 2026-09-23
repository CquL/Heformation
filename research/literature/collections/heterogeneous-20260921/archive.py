#!/usr/bin/env python3
"""Download only the 20 user-supplied references, with auditable provenance.

Run from any directory. A candidate URL must return a parseable PDF whose title
is confirmed in its opening text. Existing verified reading copies are reused.
HTML, access-denied pages and network failures are recorded, never saved as PDF.
Downloading a paper does not constitute reading or reproducing it.
"""
import concurrent.futures
import datetime
import hashlib
import json
from pathlib import Path
import re
import subprocess

import requests


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
INDEX = ROOT / "references_verified_20260921.json"
REUSE = {
    "P04": "research/literature/papers/benchmark/caric-2025.pdf",
    "P08": "research/literature/papers/formation/cat-ora-2025.pdf",
}
PREPRINT_TITLE = {
    "P04": "Cooperative Aerial Robot Inspection Challenge: A Benchmark for Heterogeneous Multi-UAV Planning and Lessons Learned",
}


def inspect_pdf(data, expected_title):
    if not data.startswith(b"%PDF-"):
        raise ValueError("response is not a PDF")
    result = subprocess.run(
        ["pdftotext", "-layout", "-", "-"], input=data,
        capture_output=True, timeout=30, check=True,
    )
    contents = result.stdout.decode("utf-8", errors="replace")
    words = set(re.findall(r"[a-z]{4,}", expected_title.lower()))
    opening = set(re.findall(r"[a-z]{4,}", contents[:18000].lower()))
    if not words or len(words & opening) / len(words) < 0.85:
        raise ValueError("PDF opening text does not match the expected title")
    return contents, {
        "sha256": hashlib.sha256(data).hexdigest(),
        "size_bytes": len(data),
        "pages": contents.count("\f"),
        "identity_check": "PDF signature, pdftotext parse, title words in opening text; human title check recorded separately",
    }


def archive(record, candidates):
    ident = record["id"]
    identity_title = PREPRINT_TITLE.get(ident, record["title"])
    folder = HERE / ident
    folder.mkdir(exist_ok=True)
    previous = folder / "record.json"
    entry = json.loads(previous.read_text()) if previous.exists() else {}
    entry.update({
        "id": ident,
        "title": record["title"],
        "doi": record["doi"],
        "arxiv": record["arxiv"],
        "official_url": record["official"],
        "code_urls": record["code"],
        "source_index": str(INDEX.relative_to(ROOT)),
        "prior_read_scope": record["read_scope"],
        "prior_code_scope": record["code_scope"],
        "mechanism_from_supplied_review": record["mechanism"],
        "boundary_from_supplied_review": record["boundary"],
        "project_use_from_supplied_review": record["project_use"],
    })
    attempts = entry.setdefault("access_attempts", [])
    current = folder / "paper.pdf"
    if current.exists():
        contents, metadata = inspect_pdf(current.read_bytes(), identity_title)
        entry.update(metadata)
        entry["integrity_rechecked_utc"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
    elif ident in REUSE:
        source = ROOT / REUSE[ident]
        data = source.read_bytes()
        contents, metadata = inspect_pdf(data, identity_title)
        current.symlink_to(Path("../../../../..") / source.relative_to(ROOT))
        entry.update(metadata, status="reused-local-pdf", reused_from=REUSE[ident])
    else:
        urls = list(candidates.get(ident, []))
        if record["arxiv"]:
            urls.append("https://arxiv.org/pdf/" + record["arxiv"])
        if record["fulltext"] and record["fulltext"] not in urls:
            urls.append(record["fulltext"])
        if not urls:
            urls = [record["official"]]
        contents = None
        tried = {a["url"] for a in attempts}
        for url in dict.fromkeys(urls):
            if url in tried:
                continue
            attempt = {"url": url, "checked_utc": datetime.datetime.now(datetime.timezone.utc).isoformat()}
            try:
                response = requests.get(url, timeout=(10, 40), headers={"User-Agent": "Mozilla/5.0"})
                attempt.update(http_status=response.status_code, final_url=response.url,
                               content_type=response.headers.get("Content-Type"))
                response.raise_for_status()
                if len(response.content) > 60_000_000:
                    raise ValueError("response exceeds 60 MB archive limit")
                contents, metadata = inspect_pdf(response.content, identity_title)
                current.write_bytes(response.content)
                entry.update(metadata, status="downloaded-open-pdf", source_url=url,
                             resolved_source_url=response.url, downloaded_utc=attempt["checked_utc"])
                attempt["result"] = "verified-pdf"
                attempts.append(attempt)
                break
            except Exception as exc:
                attempt["result"] = "unresolved"
                attempt["error"] = str(exc)
                attempts.append(attempt)
        if contents is None:
            entry["status"] = "fulltext-access-unresolved"
    if current.exists():
        entry["verified_pdf_title"] = identity_title
        entry["local_pdf"] = str(current.relative_to(ROOT))
        entry["resolved_pdf"] = str(current.resolve().relative_to(ROOT))
        (folder / "extracted.txt").write_text(contents)
        entry.setdefault("this_round_read_scope", "download and identity inspection only; selected-section review pending")
    else:
        entry["local_pdf"] = None
        entry["this_round_read_scope"] = "no fulltext; supplied review plus discovered metadata only"
    previous.write_text(json.dumps(entry, indent=2, ensure_ascii=False) + "\n")
    print(ident, entry["status"], flush=True)
    return entry


def main():
    records = json.loads(INDEX.read_text())["records"]
    path = HERE / "candidate_sources.json"
    candidates = json.loads(path.read_text()) if path.exists() else {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(lambda record: archive(record, candidates), records))
    (HERE / "manifest.json").write_text(json.dumps({
        "collection": "heterogeneous-20260921",
        "source_index": str(INDEX.relative_to(ROOT)),
        "checked_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "policy": "Public author/repository/arXiv/publisher copies only; no access-control bypass; no inferred full-paper reading or reproduction claim.",
        "records": results,
    }, indent=2, ensure_ascii=False) + "\n")


if __name__ == "__main__":
    main()
