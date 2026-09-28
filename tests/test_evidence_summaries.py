"""docs/evidence summaries: cited files exist, follow the schema and leak no local paths."""
import importlib.util
import json
from pathlib import Path
import re
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('summarize_evidence', ROOT / 'scripts/summarize_evidence.py')
summarize = importlib.util.module_from_spec(spec)
spec.loader.exec_module(summarize)


class EvidenceSummaryTests(unittest.TestCase):
    def test_every_cited_summary_exists(self):
        cited = {match for doc in (ROOT / 'docs').glob('*.md')
                 for match in re.findall(r'evidence/([a-z0-9-]+)\.json', doc.read_text())}
        self.assertTrue(cited)
        for name in sorted(cited):
            with self.subTest(name=name):
                self.assertTrue((ROOT / 'docs/evidence' / f'{name}.json').is_file())

    def test_summaries_follow_the_schema_without_local_paths(self):
        for path in sorted((ROOT / 'docs/evidence').glob('*.json')):
            with self.subTest(path=path.name):
                text = path.read_text()
                summary = json.loads(text)
                self.assertEqual(summary['schema_version'], summarize.SCHEMA_VERSION)
                self.assertEqual(summary['name'], path.stem)
                self.assertTrue(summary['sources'])
                for source in summary['sources']:
                    self.assertRegex(source['sha256'], r'^[0-9a-f]{64}$')
                    self.assertFalse(source['path'].startswith('/'), source['path'])
                    self.assertTrue('content' in source or 'tail' in source)
                self.assertNotIn('/home/', text)

    def test_compaction_drops_sample_tables_and_scrubs_paths(self):
        data = {'status': 'ok', 'samples': list(range(summarize.MAX_LIST + 1)), 'pair': [1, 2],
                'where': '/data/somebody/checkout/outputs/run-1/x.json',
                'deep': {'a': {'b': {'c': {'d': 1}}}}}
        compact = summarize.compact(data)
        self.assertEqual(compact['status'], 'ok')
        self.assertNotIn('samples', compact)
        self.assertEqual(compact['pair'], [1, 2])
        self.assertEqual(compact['where'], 'outputs/run-1/x.json')
        # Three dictionary levels are kept, the top level included.
        self.assertEqual(compact['deep'], {'a': {}})

    def test_script_writes_a_named_summary_with_digests(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory) / 'outputs' / 'run-1'
            run.mkdir(parents=True)
            (run / 'run_metrics.json').write_text(json.dumps({'status': 'completed', 'gates_passed': 3}))
            (run / 'notes.txt').write_text('ignored: not JSON in a directory\n')
            out = Path(directory) / 'evidence'
            summarize.main(['--name', 'demo-run', '--output-dir', str(out), str(run)])
            summary = json.loads((out / 'demo-run.json').read_text())
            self.assertEqual([s['path'] for s in summary['sources']], ['outputs/run-1/run_metrics.json'])
            self.assertEqual(summary['sources'][0]['content'], {'status': 'completed', 'gates_passed': 3})
            with self.assertRaises(SystemExit):
                summarize.main(['--name', 'Bad Name', '--output-dir', str(out), str(run)])


if __name__ == '__main__':
    unittest.main()
