"""Run report: self-contained HTML with well-formed inline SVG, without ROS."""
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "njord_sim"))
from njord_sim.report_core import read_timeseries, render_report, write_report
from njord_sim.scenario_core import RaceScorer, load_scenario
from njord_sim.setpoint_core import SetpointCourse

HEADER = "t_s,x_m,y_m,yaw_deg,surge_mps,sway_mps,yaw_rate_dps,setpoint_index,distance_m,heading_error_deg"


def setpoint_run():
    scenario = load_scenario(ROOT / "scenarios/goto_square.yaml")
    scenario["setpoints"][0]["name"] = "<east & co>"
    scorer = SetpointCourse(scenario)
    rows = [HEADER + ",force_1_n,force_2_n"]
    for i in range(400):
        t = i * 0.5
        x = min(20.0, t)
        scorer.update(t, x, 0.0, 0.0, force_sum=200.0)
        track = scorer.active
        rows.append(f"{t},{x},0,0,1,0,0,{track.index if track else ''},"
                    f"{track.distance if track else ''},{1.5 if track else ''},100,100")
    return scorer.metrics(), "\n".join(rows) + "\n"


class ReportTests(unittest.TestCase):
    def check(self, page):
        self.assertTrue(page.startswith("<!doctype html>"))
        self.assertNotRegex(page, r"\b(nan|inf)\b")
        self.assertNotIn("<script", page)
        self.assertNotRegex(page, r'(src|href)="https?:')
        svgs = re.findall(r"<svg.*?</svg>", page, flags=re.S)
        self.assertTrue(svgs)
        for svg in svgs:
            ET.fromstring(svg)  # well-formed
        return svgs

    def test_setpoint_report_has_map_table_and_charts(self):
        metrics, csv_text = setpoint_run()
        with tempfile.TemporaryDirectory() as directory:
            series = Path(directory) / "timeseries.csv"
            series.write_text(csv_text)
            rows = read_timeseries(series)
        page = render_report(metrics, rows)
        svgs = self.check(page)
        self.assertGreaterEqual(len(svgs), 5)  # map, distance, heading, speed, thrust
        self.assertIn("&lt;east &amp; co&gt;", page)
        self.assertIn("Targets reached", page)
        self.assertIn("thruster 2", page)
        self.assertIn("timeout", page)

    def test_gate_race_report_without_setpoints(self):
        scenario = load_scenario(ROOT / "scenarios/reference.yaml")
        scorer = RaceScorer(scenario)
        for i in range(50):
            scorer.update(i * 0.2, i * 0.5, 0.0, 0.0)
        metrics = {**scorer.metrics(), "scenario": scenario, "course": "gates"}
        rows = [{"t_s": i * 0.2, "x_m": i * 0.5, "y_m": 0.0, "surge_mps": 2.5, "sway_mps": 0.0} for i in range(50)]
        page = render_report(metrics, rows)
        self.check(page)
        self.assertIn("Gates passed", page)

    def test_empty_run_and_nonfinite_values_still_render(self):
        metrics = {"status": "wall_timeout", "time_s": float("nan"), "scenario": {}}
        page = render_report(metrics, [])
        self.assertIn("No track recorded", page)
        self.assertNotRegex(page, r"\bnan\b")

    def test_cli_rebuilds_the_report_on_the_host(self):
        metrics, csv_text = setpoint_run()
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            (run / "run_metrics.json").write_text(json.dumps(metrics, allow_nan=False))
            (run / "timeseries.csv").write_text(csv_text)
            result = subprocess.run([sys.executable, str(ROOT / "scripts/make_report.py"), str(run)],
                                    capture_output=True, text=True, check=True)
            self.assertEqual(Path(result.stdout.strip()), run / "report.html")
            self.check((run / "report.html").read_text())
            self.assertFalse((run / "report.html.tmp").exists())
            write_report(run / "run_metrics.json", run / "missing.csv", run / "second.html")
            self.assertIn("Targets", (run / "second.html").read_text())


if __name__ == "__main__":
    unittest.main()
