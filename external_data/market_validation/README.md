# Official AEMO Validation Sources

The generated `source_manifest.json` pins URLs, byte counts and SHA256 for
seven DispatchIS daily archives (2026-08-28 through 2026-09-03), one next-day
unit dispatch archive (market date 2026-09-01), four dictionary pages, and
the seven bid archives retained in `../market_bids/`. Downloaded 2026-09-07.
ZIPs are excluded from Git; preserve local archives because CURRENT URLs expire.

```bash
python3 tools/market_validation/aemo_data.py --download
```

The command retrieves missing files and rejects changes to recorded hashes.
It does not replace an expired archive with a different date or silently
refresh a changed dictionary. Regional dispatch uses file timestamps ending
00:05 through the next 00:00; next-day unit dispatch uses the AEMO market-day
clock. Noon bid quantities must explicitly have `INTERVAL_DATETIME=12:00:00`.
No conversion to China local time is implied.

Official directories:

- https://nemweb.com.au/Reports/ARCHIVE/DispatchIS_Reports/
- https://nemweb.com.au/Reports/CURRENT/Next_Day_Dispatch/
- https://nemweb.com.au/Reports/CURRENT/Bidmove_Complete/
- https://nemweb.com.au/Reports/CURRENT/MMSDataModelReport/Electricity/Electricity%20Data%20Model%20Report_files/Elec20.htm
- https://nemweb.com.au/Reports/CURRENT/MMSDataModelReport/Electricity/Electricity%20Data%20Model%20Report_files/Elec20_1.htm
- https://nemweb.com.au/Reports/CURRENT/MMSDataModelReport/Electricity/Electricity%20Data%20Model%20Report_files/Elec22.htm

These sources support public offer/dispatch identities and explicitly scaled
input-transfer experiments. Regional RRP is not a nodal LMP; capacity FCAS bids
are not Yunnan mileage bids; DUID is not a company. The 25 observed FCAS
award/actual-availability discrepancies are retained as unresolved field
semantics, not discarded or used to certify a universal availability bound.
See `docs/modules/market/aemo_validation.md` for evidence and coverage limits.
