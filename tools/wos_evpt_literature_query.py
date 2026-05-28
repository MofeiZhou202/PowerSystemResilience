#!/usr/bin/env python3
"""Query Web of Science for EV power-traffic coupling literature.

The script intentionally has no third-party dependencies. It expects a
Clarivate/Web of Science API key in one of these environment variables:
WOS_API_KEY, WEB_OF_SCIENCE_API_KEY, or CLARIVATE_API_KEY.

The default endpoint targets the Web of Science Starter API. Override it with
--endpoint or WOS_API_ENDPOINT if your institution uses a different API tier.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import urllib.parse
import urllib.request
from typing import Any, Dict, Iterable, List


DEFAULT_ENDPOINT = "https://api.clarivate.com/apis/wos-starter/v1/documents"
DEFAULT_QUERY = (
    '("electric vehicle" OR EV OR V2G OR "vehicle-to-grid") '
    'AND ("cell transmission model" OR CTM OR "dynamic traffic assignment") '
    'AND ("optimal power flow" OR OPF OR "power grid") '
    'AND ("smart charging" OR "charging station" OR "charging stations")'
)


def api_key() -> str:
    for name in ("WOS_API_KEY", "WEB_OF_SCIENCE_API_KEY", "CLARIVATE_API_KEY"):
        value = os.environ.get(name, "").strip()
        if value:
            return value
    return ""


def normalize_records(payload: Dict[str, Any]) -> List[Dict[str, str]]:
    candidates: Iterable[Any]
    if isinstance(payload.get("hits"), list):
        candidates = payload["hits"]
    elif isinstance(payload.get("documents"), list):
        candidates = payload["documents"]
    elif isinstance(payload.get("data"), list):
        candidates = payload["data"]
    else:
        candidates = []

    records: List[Dict[str, str]] = []
    for item in candidates:
        if not isinstance(item, dict):
            continue
        source = item.get("source") if isinstance(item.get("source"), dict) else item
        title = source.get("title") or source.get("displayTitle") or ""
        year = source.get("year") or source.get("pubyear") or source.get("publicationYear") or ""
        journal = source.get("sourceTitle") or source.get("journal") or source.get("source") or ""
        doi = source.get("doi") or source.get("DOI") or ""
        uid = source.get("uid") or source.get("UT") or item.get("uid") or ""
        records.append(
            {
                "uid": str(uid),
                "year": str(year),
                "title": str(title),
                "journal": str(journal),
                "doi": str(doi),
            }
        )
    return records


def write_csv(path: str, records: List[Dict[str, str]]) -> None:
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["uid", "year", "title", "journal", "doi"])
        writer.writeheader()
        writer.writerows(records)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--query", default=DEFAULT_QUERY, help="Web of Science query string")
    parser.add_argument("--endpoint", default=os.environ.get("WOS_API_ENDPOINT", DEFAULT_ENDPOINT))
    parser.add_argument("--limit", type=int, default=50)
    parser.add_argument("--page", type=int, default=1)
    parser.add_argument("--json-out", default="docs/technical_notebook/wos_evpt_records.json")
    parser.add_argument("--csv-out", default="docs/technical_notebook/wos_evpt_records.csv")
    parser.add_argument("--dry-run", action="store_true", help="Print request metadata without calling the API")
    args = parser.parse_args()

    params = {"q": args.query, "limit": str(args.limit), "page": str(args.page)}
    url = args.endpoint + "?" + urllib.parse.urlencode(params)

    if args.dry_run:
        print("endpoint:", args.endpoint)
        print("query:", args.query)
        print("url:", url)
        return 0

    key = api_key()
    if not key:
        print(
            "No Web of Science API key found. Set WOS_API_KEY, "
            "WEB_OF_SCIENCE_API_KEY, or CLARIVATE_API_KEY in the shell.",
            file=sys.stderr,
        )
        return 2

    request = urllib.request.Request(url, headers={"X-ApiKey": key, "Accept": "application/json"})
    with urllib.request.urlopen(request, timeout=60) as response:
        payload = json.loads(response.read().decode("utf-8"))

    records = normalize_records(payload)
    os.makedirs(os.path.dirname(args.json_out), exist_ok=True)
    with open(args.json_out, "w", encoding="utf-8") as handle:
        json.dump({"query": args.query, "records": records, "raw": payload}, handle, indent=2)
    write_csv(args.csv_out, records)
    print(f"wrote {len(records)} records")
    print(args.json_out)
    print(args.csv_out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
