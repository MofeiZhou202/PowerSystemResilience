# AEMO Public Bid Snapshots

Official source: https://nemweb.com.au/Reports/Current/Bidmove_Complete/

Seven daily ZIP files, 2026-08-28 through 2026-09-03, downloaded 2026-09-07.
These are actual public ENERGY/FCAS offer records, not dispatch prices and
not Southern China market bids. Raw ZIPs are retained locally but excluded
from Git. `tools/market_bid_empirical.py --download` downloads missing files
and records their URLs, hashes, sizes and local timestamps in
`output/market-bids/manifest.json`. The CURRENT listing is rolling; preserve
the local archives for later reproduction when upstream files expire.

Official dictionary (downloaded alongside the archives):
https://www.nemweb.com.au/Reports/CURRENT/MMSDataModelReport/Electricity/Electricity%20Data%20Model%20Report_files/Elec10.htm

`BIDDAYOFFER_D` describes the latest accepted bids applicable to dispatch,
published after 04:00 the next day. Its key includes SETTLEMENTDATE, DUID,
BIDTYPE and DIRECTION; BIDSETTLEMENTDATE denotes the submitted market date
and is not the join key. `BIDPEROFFER_D` is its child with an interval key.
Daily price and interval quantity OFFERDATE need not coincide.

The experiment extracts ENERGY at PERIODID 96 (interval ending 12:00 in
the file clock), separates GEN from LOAD, preserves signed AUD/MWh prices
and zero-availability exclusions, and uses the final snapshots only.
It cannot reconstruct what a participant knew at an earlier bid cutoff.
Public release delay also prevents calling chronological splits a live
day-ahead forecasting backtest.

See `docs/modules/market/empirical_bidding_analysis.md` for the complete
transformation, validation protocol, empirical results and limitations.
