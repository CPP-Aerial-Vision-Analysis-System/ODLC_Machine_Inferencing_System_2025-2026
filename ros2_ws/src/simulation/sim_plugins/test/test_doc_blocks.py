"""
sim_plugins/test/test_doc_blocks.py -- PURPOSE: keep docs/simulation/PLUGINS.md in step with the code. The page's
profile reference and contract tables are GENERATED (doc_blocks.py) between marker lines; these tests check that
  * the generated parts mention every profile key, every entry key and every contract topic / service;
  * replacing them leaves the hand-written text around them untouched, is repeatable, and reports missing markers;
  * the real PLUGINS.md is up to date (if this fails: python3 -m sim_plugins.profile_tool docs --write).
"""

from sim_plugins import contracts, doc_blocks, profile_tool, schema

from conftest import REPO

DOC = REPO / 'docs' / 'simulation' / 'PLUGINS.md'


def test_reference_lists_every_key_with_its_default():
    text = doc_blocks.reference_block()
    for dotted, field in schema.all_keys():
        assert f'| `{dotted}` |' in text, dotted
    for key in schema.ENTRY_KEYS:
        assert f'| `{key}` |' in text, key
    assert '| `camera.model` | `zr10` | one of `zr10`, `a8mini`' in text
    assert '| `network.mavros_port` | `14552` | a port number 1..65535' in text


def test_contracts_block_has_an_anchor_and_every_item_per_slot():
    text = doc_blocks.contracts_block()
    assert all(line == line.rstrip() for line in text.splitlines())
    for slot, c in contracts.CONTRACTS.items():
        assert f'<a id="{slot}"></a>' in text and f'### {c.title}' in text
        for item in list(c.topics) + list(c.services):
            assert f'`{item.name}`' in text, item.name
    assert 'publishes' in text and 'publishs' not in text


def test_replace_keeps_the_hand_written_text_and_is_repeatable():
    page = ('# Title\nhand-written intro\n\n'
            f'{doc_blocks.begin_marker("reference")}\nold stuff\n{doc_blocks.end_marker("reference")}\n\n'
            'middle text\n\n'
            f'{doc_blocks.begin_marker("contracts")}\n{doc_blocks.end_marker("contracts")}\n\nend text\n')
    new, changed, missing = doc_blocks.replace_blocks(page)
    assert changed == ['reference', 'contracts'] and missing == []
    assert new.startswith('# Title\nhand-written intro\n') and 'middle text' in new and new.endswith('end text\n')
    assert 'old stuff' not in new and '| `world.name` |' in new
    again, changed2, _ = doc_blocks.replace_blocks(new)
    assert again == new and changed2 == [] and doc_blocks.out_of_date(new) == []


def test_missing_markers_are_reported():
    _, _, missing = doc_blocks.replace_blocks('# no markers here\n')
    assert missing == list(doc_blocks.BLOCKS)


def test_docs_command_check_and_write(tmp_path, capsys):
    page = tmp_path / 'P.md'
    page.write_text(f'x\n{doc_blocks.begin_marker("reference")}\n{doc_blocks.end_marker("reference")}\n'
                    f'{doc_blocks.begin_marker("contracts")}\n{doc_blocks.end_marker("contracts")}\n')
    assert profile_tool.main(['docs', '--check', '--file', str(page)]) == 1
    assert 'out of date' in capsys.readouterr().err
    assert profile_tool.main(['docs', '--write', '--file', str(page)]) == 0
    assert b'\r\n' not in page.read_bytes()
    assert profile_tool.main(['docs', '--check', '--file', str(page)]) == 0


def test_the_real_plugins_md_is_up_to_date():
    assert DOC.is_file(), 'docs/simulation/PLUGINS.md is missing'
    stale = doc_blocks.out_of_date(DOC.read_text(encoding='utf-8'))
    assert stale == [], (f'{stale} out of date in docs/simulation/PLUGINS.md: run '
                         'python3 -m sim_plugins.profile_tool docs --write')
