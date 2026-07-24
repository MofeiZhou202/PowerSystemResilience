import copy
import csv
import hashlib
import unittest
from datetime import date, timedelta
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Optional, Tuple

import climate_data
import cftime
import numpy as np


class ClimateDataContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.study = climate_data.load_json(climate_data.DEFAULT_STUDY)
        cls.sources = climate_data.load_json(climate_data.DEFAULT_SOURCES)

    def test_shaanxi_pilot_contract_is_valid(self):
        report = climate_data.validate_contract(self.study, self.sources)
        self.assertTrue(report["valid"], report["errors"])
        self.assertEqual(report["counts"]["zones"], 3)
        self.assertEqual(report["counts"]["sites"], 4)
        self.assertEqual(report["counts"]["pilot_gcms"], 5)
        self.assertEqual(report["counts"]["outer_evaluation_gcms"], 6)
        self.assertEqual(report["counts"]["all_gcms"], 11)

    def test_model_sets_are_disjoint_and_realizations_are_model_specific(self):
        pilot = climate_data.models_for_set(self.study, "pilot")
        outer = climate_data.models_for_set(self.study, "outer")
        self.assertFalse(set(pilot) & set(outer))
        self.assertEqual(climate_data.models_for_set(self.study, "all"), pilot + outer)
        self.assertEqual(
            climate_data.nex_realization(self.study, "CNRM-CM6-1"), "r1i1p1f2"
        )

    def test_outer_model_is_rejected_by_default_pilot_gate(self):
        with self.assertRaises(climate_data.ContractError):
            climate_data.require_model_in_set(self.study, "MIROC6", "pilot")
        climate_data.require_model_in_set(self.study, "MIROC6", "outer")

    def test_nex_metadata_repair_is_exact_and_rejects_explicit_conflicts(self):
        source = climate_data.source_index(self.sources)["nex_gddp_cmip6"]
        units, repair = climate_data.resolve_nex_native_units(
            source, "FGOALS-g3", "tasmin", ""
        )
        self.assertEqual(units, "K")
        self.assertEqual(repair["id"], "nex_v2_fgoals_g3_tasmin_missing_units_k")
        units, repair = climate_data.resolve_nex_native_units(
            source, "FGOALS-g3", "tasmin", "K"
        )
        self.assertEqual(units, "K")
        self.assertIsNone(repair)
        units, repair = climate_data.resolve_nex_native_units(
            source, "FGOALS-g3", "tasmin", "degC"
        )
        self.assertEqual(units, "degC")
        self.assertIsNone(repair)
        units, repair = climate_data.resolve_nex_native_units(
            source, "MIROC6", "tasmin", ""
        )
        self.assertEqual(units, "")
        self.assertIsNone(repair)

    def test_site_outside_zone_is_rejected(self):
        study = copy.deepcopy(self.study)
        study["spatial_design"]["zones"][0]["sites"][0]["longitude"] = 100.0
        report = climate_data.validate_contract(study, self.sources)
        self.assertFalse(report["valid"])
        self.assertTrue(any("outside" in error for error in report["errors"]))

    def test_default_plan_is_point_scoped(self):
        plan = climate_data.build_plan(
            self.study, include_optional=False, include_extension=False
        )
        era = [row for row in plan["records"] if row["source"] == "era5_land"]
        nex = [row for row in plan["records"] if row["source"] == "nex_gddp_cmip6"]
        self.assertEqual(len(era), 4 * 4)
        self.assertEqual(len(nex), 5 * 3 * 6)
        self.assertEqual(plan["estimate"]["nex_corridor_annual_request_count"], 2700)
        self.assertEqual(plan["estimate"]["nex_equivalent_point_request_count"], 10800)
        self.assertTrue(all("site=" in row["target"] for row in era))
        self.assertTrue(all(row["period"] == "1985-2014" for row in era))
        self.assertTrue(
            all(row["sites"] == ["yulin", "xian", "hanzhong", "ankang"] for row in nex)
        )

    def test_outer_plan_does_not_include_pilot_models(self):
        plan = climate_data.build_plan(
            self.study,
            include_optional=False,
            include_extension=False,
            model_set="outer",
        )
        nex = [row for row in plan["records"] if row["source"] == "nex_gddp_cmip6"]
        self.assertEqual(len(nex), 6 * 3 * 6)
        self.assertEqual(plan["configuration"]["model_set"], "outer")
        self.assertEqual(
            {row["model"] for row in nex},
            set(self.study["climate_states"]["outer_evaluation_gcms"]),
        )

    def test_era_request_has_exact_point_and_annual_period(self):
        site = next(climate_data.iter_sites(self.study))
        request = climate_data.era5_timeseries_request(
            site,
            1985,
            ["2m_temperature", "2m_dewpoint_temperature"],
        )
        self.assertEqual(request["location"]["longitude"], 109.7346)
        self.assertEqual(request["location"]["latitude"], 38.2854)
        self.assertEqual(request["date"], ["1985-01-01/1985-12-31"])
        self.assertIn("2m_temperature", request["variable"])
        self.assertNotIn("volumetric_soil_water_level_1", request["variable"])

    def test_expected_days_respects_declared_model_calendar(self):
        self.assertEqual(
            climate_data.expected_days_in_calendar_year(1988, "proleptic_gregorian"),
            366,
        )
        self.assertEqual(
            climate_data.expected_days_in_calendar_year(1988, "365_day"), 365
        )
        self.assertEqual(
            climate_data.expected_days_in_calendar_year(1988, "360_day"), 360
        )

    def test_time_comparison_preserves_non_gregorian_calendar(self):
        values = np.asarray(
            [
                cftime.Datetime360Day(1985, 2, 29, 12),
                cftime.Datetime360Day(1985, 2, 30, 12),
            ],
            dtype=object,
        )
        self.assertTrue(climate_data.calendar_time_values_equal(values, values, np))
        noleap = np.asarray(
            [
                cftime.DatetimeNoLeap(1985, 2, 28, 12),
                cftime.DatetimeNoLeap(1985, 3, 1, 12),
            ],
            dtype=object,
        )
        self.assertFalse(
            climate_data.calendar_time_values_equal(values, noleap, np)
        )

    def test_relative_humidity_clipping_is_explicit_and_counted(self):
        clipped, count = climate_data.clip_relative_humidity(
            np.asarray([-0.2, 50.0, 100.0, 101.3]), np
        )
        self.assertEqual(clipped.tolist(), [0.0, 50.0, 100.0, 100.0])
        self.assertEqual(count, 2)

    def test_era_required_variables_follow_timeseries_storage_groups(self):
        groups = climate_data.era5_timeseries_groups(self.study, include_optional=False)
        self.assertEqual(
            groups,
            {
                "temperature_humidity": [
                    "2m_temperature",
                    "2m_dewpoint_temperature",
                ],
                "precipitation": ["total_precipitation"],
                "solar_radiation": ["surface_solar_radiation_downwards"],
                "wind": [
                    "10m_u_component_of_wind",
                    "10m_v_component_of_wind",
                ],
            },
        )

    def test_bulk_nex_extent_must_contain_all_sites(self):
        study = copy.deepcopy(self.study)
        study["spatial_design"]["bulk_nex_request_extent"] = [108.0, 33.0, 109.0, 35.0]
        report = climate_data.validate_contract(study, self.sources)
        self.assertFalse(report["valid"])
        self.assertTrue(
            any(
                "outside bulk NEX request extent" in error for error in report["errors"]
            )
        )

    def test_nex_catalog_resolution_and_point_url(self):
        xml = b'<catalog><dataset urlPath="AMES/example/pr_1985_v2.0.nc"/></catalog>'
        path = climate_data.resolve_nex_dataset(xml, 1985, "2.0")
        site = next(
            site for site in climate_data.iter_sites(self.study) if site["id"] == "xian"
        )
        url = climate_data.nex_point_request_url(path, "pr", site, 1985)
        self.assertIn("pr_1985_v2.0.nc?", url)
        self.assertIn("latitude=34.3416", url)
        self.assertIn("longitude=108.9398", url)
        self.assertIn("time_start=1985-01-01T12%3A00%3A00Z", url)

    def test_nex_corridor_url_uses_registered_bbox_and_full_year(self):
        xml = b'<catalog><dataset urlPath="AMES/example/pr_1985_v2.0.nc"/></catalog>'
        path = climate_data.resolve_nex_dataset(xml, 1985, "2.0")
        bbox = self.study["spatial_design"]["bulk_nex_request_extent"]
        url = climate_data.nex_corridor_request_url(path, "pr", bbox, 1985)
        self.assertIn("north=38.4", url)
        self.assertIn("west=106.9", url)
        self.assertIn("east=109.8", url)
        self.assertIn("south=32.6", url)
        self.assertIn("time_end=1985-12-31T12%3A00%3A00Z", url)
        self.assertIn("disableProjSubset=on", url)

    def test_360_day_nex_request_ends_on_december_30(self):
        xml = b'<catalog><dataset urlPath="AMES/example/pr_1985_v2.0.nc"/></catalog>'
        path = climate_data.resolve_nex_dataset(xml, 1985, "2.0")
        study = copy.deepcopy(self.study)
        study["climate_states"]["nex_request_calendar_overrides"] = {
            "synthetic-360-day-model": "360_day"
        }
        calendar_name = climate_data.nex_request_calendar(
            study, "synthetic-360-day-model"
        )
        self.assertEqual(calendar_name, "360_day")
        url = climate_data.nex_corridor_request_url(
            path,
            "pr",
            self.study["spatial_design"]["bulk_nex_request_extent"],
            1985,
            calendar_name,
        )
        self.assertIn("time_end=1985-12-30T12%3A00%3A00Z", url)
        self.assertNotIn("time_end=1985-12-31", url)

    def test_boolean_runs_report_half_open_intervals(self):
        self.assertEqual(
            climate_data.boolean_run_lengths(
                [False, True, True, False, True, True, True]
            ),
            [(1, 3, 2), (4, 7, 3)],
        )
        self.assertEqual(climate_data.maximum_run_length([False, False]), 0)
        self.assertEqual(climate_data.maximum_run_length([True, True, False]), 2)

    def test_theil_sen_slope_uses_pairwise_median(self):
        self.assertEqual(
            climate_data.theil_sen_slope([2000, 2001, 2002], [1.0, 3.0, 5.0]),
            2.0,
        )
        self.assertTrue(np.isnan(climate_data.theil_sen_slope([2000], [float("nan")])))

    def test_masked_mean_is_blank_when_no_event_days_exist(self):
        values = np.asarray([20.0, 24.0, 28.0])
        self.assertEqual(
            climate_data.masked_mean_or_blank(
                values, np.asarray([False, False, False]), np
            ),
            "",
        )
        self.assertEqual(
            climate_data.masked_mean_or_blank(
                values, np.asarray([False, True, True]), np
            ),
            26.0,
        )

    def test_noleap_day_mapping_assigns_february_29_to_february_28(self):
        self.assertEqual(
            climate_data.noleap_day_indices([1, 2, 2, 3, 12], [1, 28, 29, 1, 31]),
            [0, 58, 58, 59, 364],
        )

    def test_calendar_day_percentile_uses_circular_window_and_excludes_leap_day(self):
        dates = [date(2001, 1, 1) + timedelta(days=offset) for offset in range(365)]
        dates.append(date(2000, 2, 29))
        months = np.asarray([value.month for value in dates], dtype=int)
        days = np.asarray([value.day for value in dates], dtype=int)
        day_indices = np.asarray(
            climate_data.noleap_day_indices(months, days), dtype=int
        )
        values = np.asarray([*range(365), 999.0], dtype=float)
        thresholds = climate_data.calendar_day_percentile_thresholds(
            values, day_indices, months, days, 0.5, 3, np
        )
        self.assertEqual(thresholds[0], 1.0)
        self.assertEqual(thresholds[58], 58.0)

    def test_hourly_heat_stress_formulas_use_concurrent_inputs(self):
        temperature = np.asarray([30.0])
        dewpoint = np.asarray([20.0])
        humidity = climate_data.relative_humidity_from_temperature_dewpoint(
            temperature, dewpoint, np
        )
        wet_bulb = climate_data.stull_wet_bulb_temperature(temperature, humidity, np)
        apparent = climate_data.shade_apparent_temperature(
            temperature, dewpoint, np.asarray([2.0]), np
        )
        self.assertAlmostEqual(float(humidity[0]), 55.0, delta=0.2)
        self.assertAlmostEqual(float(wet_bulb[0]), 23.17, delta=0.05)
        self.assertAlmostEqual(float(apparent[0]), 32.32, delta=0.05)

    def test_daily_maximum_preserves_context_at_extreme_hour(self):
        metric = np.arange(48, dtype=float)
        metric[30] = np.nan
        temperature = metric + 100.0
        result = climate_data.daily_maximum_with_context(
            metric, {"temperature": temperature}, np
        )
        self.assertEqual(result["hour_utc"].tolist(), [23, 23])
        self.assertEqual(result["maximum"].tolist(), [23.0, 47.0])
        self.assertEqual(result["temperature"].tolist(), [123.0, 147.0])
        self.assertEqual(result["valid_hours"].tolist(), [24, 23])

    def write_observation_fixture(
        self, root: Path, unapproved_year: Optional[int] = None
    ) -> Tuple[Path, dict]:
        path = root / "xian_observations.csv"
        fieldnames = [
            "date",
            "location_id",
            "longitude",
            "latitude",
            "elevation_m",
            "tasmax",
            "tasmin",
            "pr",
            "quality_flag",
        ]
        with path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            cursor = date(1995, 1, 1)
            end = date(2014, 12, 31)
            while cursor <= end:
                writer.writerow(
                    {
                        "date": cursor.isoformat(),
                        "location_id": "CMA_TEST_XIAN",
                        "longitude": 108.9,
                        "latitude": 34.3,
                        "elevation_m": 410.0,
                        "tasmax": 25.0,
                        "tasmin": 15.0,
                        "pr": 1.0,
                        "quality_flag": "X" if cursor.year == unapproved_year else "A",
                    }
                )
                cursor += timedelta(days=1)
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        definition = climate_data.load_json(climate_data.DEFAULT_OBSERVATION_VALIDATION)
        provenance = {
            "source_id": "cma_station",
            "dataset_title": "Synthetic test observations",
            "provider": "unit-test",
            "release_or_version": "v1",
            "station_or_grid_ids": ["CMA_TEST_XIAN"],
            "target_site": "xian",
            "order_or_access_identifier": "test-order",
            "download_date": "2026-07-24",
            "license_or_terms": "test-only",
            "file_sha256": digest,
            "canonical_units": definition["canonical_units"],
            "approved_quality_flags": ["A"],
            "quality_flag_meanings": {"A": "provider approved", "X": "rejected"},
            "transformation_history": ["synthetic fixture"],
            "reference_grid_elevation_m": 405.0,
            "reference_grid_elevation_source": "synthetic fixture",
        }
        return path, provenance

    def test_observation_admission_accepts_twenty_complete_approved_years(self):
        with TemporaryDirectory() as directory:
            path, provenance = self.write_observation_fixture(Path(directory))
            report = climate_data.validate_observation_table(
                path,
                "cma_station",
                "xian",
                self.study,
                climate_data.load_json(climate_data.DEFAULT_OBSERVATION_VALIDATION),
                provenance,
                {"longitude": 108.9, "latitude": 34.3},
            )
        self.assertTrue(report["valid"], report["errors"])
        self.assertEqual(len(report["time"]["years_passing_annual_coverage"]), 20)
        self.assertEqual(report["location"]["distance_to_era5_grid_km"], 0.0)

    def test_observation_admission_rejects_year_with_unapproved_flags(self):
        with TemporaryDirectory() as directory:
            path, provenance = self.write_observation_fixture(
                Path(directory), unapproved_year=2000
            )
            report = climate_data.validate_observation_table(
                path,
                "cma_station",
                "xian",
                self.study,
                climate_data.load_json(climate_data.DEFAULT_OBSERVATION_VALIDATION),
                provenance,
                {"longitude": 108.9, "latitude": 34.3},
            )
        self.assertFalse(report["valid"])
        self.assertTrue(
            any("19 calendar years" in error for error in report["errors"]),
            report["errors"],
        )

    def test_observation_admission_rejects_provenance_hash_mismatch(self):
        with TemporaryDirectory() as directory:
            path, provenance = self.write_observation_fixture(Path(directory))
            provenance["file_sha256"] = "0" * 64
            report = climate_data.validate_observation_table(
                path,
                "cma_station",
                "xian",
                self.study,
                climate_data.load_json(climate_data.DEFAULT_OBSERVATION_VALIDATION),
                provenance,
                {"longitude": 108.9, "latitude": 34.3},
            )
        self.assertFalse(report["valid"])
        self.assertIn(
            "Observation provenance file_sha256 does not match the CSV",
            report["errors"],
        )


if __name__ == "__main__":
    unittest.main()
