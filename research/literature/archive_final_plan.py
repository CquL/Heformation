#!/usr/bin/env python3
"""Archive only the frozen bibliography; reuse verified local copies.

No reading claim is inferred from successful download. Publisher HTML/error
responses are recorded as access failures, never stored as PDF files.
"""
import argparse
import concurrent.futures
import hashlib
import json
from pathlib import Path
import re
import subprocess
import urllib.request
import yaml

ROOT = Path(__file__).resolve().parent
REUSE = dict(zip(
    ['P01','P04','P05','P08','P09','P10','P12','P15','P16','P27','P28'],
    ['calvo-2024-delay-propagation','apex-mr-2025','hltl-gcs-2025',
     'swarm-formation-icra2022','swarm-formation-2022','minco-2022',
     'cocoplan-2026','robust-mader-2023','fastrack-2017','caric-2025','xiroi-ii-2023']))
KEYS = {'P02':'grstaps-2021','P03':'d-itags-2023','P06':'dai-heterogeneous-mrta-2025',
        'P07':'dhrta-2020','P11':'cat-ora-2025','P13':'guo-zavlanos-2018',
        'P14':'achord-2022','P17':'hybrid-smooth-transition-2021','P18':'tj-flyingfish-2023',
        'P19':'dipper-2021','P20':'nezha-iv-2024','P21':'wukong-2025',
        'P22':'hauv-adsc-2020','P23':'surfing-transition-2023',
        'P24':'aerial-aquatic-hitchhiking-2022','P25':'transition-survey-2026',
        'P26':'wukong-omni-2026','P29':'joe-2022-marine-heterogeneous-collaboration',
        'P30':'seaclear-2026','P31':'stonefish-2025','P32':'average-dwell-time-1999'}
EXTRA = {
    'P02':['https://journals.sagepub.com/doi/pdf/10.1177/02783649211052066'],
    'P07':['https://ojs.aaai.org/index.php/ICAPS/article/download/6738/6592'],
    'P19':['https://nubot-uaav.github.io/static/pdf/folding_wing_aavs/Dipper%20A%20Dynamically%20Transitioning.pdf'],
    'P30':['https://research.tudelft.nl/files/267348972/1-s2.0-S0952197626003751-main.pdf'],
    'P32':['https://web.ece.ucsb.edu/~hespanha/published/avedwell.pdf'],
}

def inspect_pdf(data, title):
    if not data.startswith(b'%PDF'):
        raise ValueError('response is not a PDF')
    result = subprocess.run(['pdftotext', '-', '-'], input=data, capture_output=True, timeout=20)
    if result.returncode:
        raise ValueError('PDF text extraction failed')
    extracted = result.stdout.decode(errors='replace')
    words = set(re.findall(r'[a-z]{4,}', title.lower())) - {'with','from','under','using','based','robot','robots'}
    present = set(re.findall(r'[a-z]{4,}', extracted[:18000].lower()))
    if not words or len(words & present) / len(words) < .65:
        raise ValueError('PDF title identity could not be confirmed')
    return {'pdf_size_bytes':len(data), 'pdf_sha256':hashlib.sha256(data).hexdigest(),
            'pdf_pages':extracted.count('\f'), 'identity_check':'title vocabulary in extracted opening text'}

def archive(ref, existing):
    ident = ref['id']
    if ident == 'T01':
        return {'id':ident,'status':'local-source-reused','path':'upstream/Fossen'}
    key = REUSE.get(ident, KEYS.get(ident))
    entry = dict(existing.get(key, {}))
    entry.update(key=key, final_plan_id=ident)
    if entry.get('pdf_file') and (ROOT / entry['pdf_file']).is_file():
        data = (ROOT / entry['pdf_file']).read_bytes()
        digest = hashlib.sha256(data).hexdigest()
        if not data.startswith(b'%PDF') or digest != entry.get('pdf_sha256'):
            raise ValueError('Existing PDF integrity mismatch: ' + key)
        return {'id':ident,'status':'reused','entry':entry}
    category = ('mrta' if ident in ['P02','P03','P06','P07'] else
                'formation' if ident == 'P11' else
                'task_motion' if ident in ['P13','P14'] else
                'safety' if ident == 'P32' else 'maritime')
    entry.update(title=ref['title'], category=category,
                 url=ref['official_or_author_url'])
    entry.setdefault('venue', ref['venue'])
    entry.setdefault('read_scope', 'not-read')
    entry['source_index'] = 'Heformation_最终方案与文献_20260919/references.json'
    attempts = []
    urls = EXTRA.get(ident, []) + re.findall(r'https?://[^\s<>]+', ref['pdf_or_fulltext_url'])
    for url in dict.fromkeys(urls):
        try:
            request = urllib.request.Request(url, headers={'User-Agent':'Mozilla/5.0'})
            with urllib.request.urlopen(request, timeout=20) as response:
                data = response.read(35_000_001)
                final_url = response.url
            if len(data) > 35_000_000:
                raise ValueError('response exceeds 35 MB archive limit')
            metadata = inspect_pdf(data, ref['title'])
            path = Path('papers') / category / (key + '.pdf')
            (ROOT / path).parent.mkdir(parents=True, exist_ok=True)
            (ROOT / path).write_bytes(data)
            entry.update(metadata, pdf_file=str(path), pdf_status='downloaded',
                         downloaded_from=final_url, downloaded_on='2026-09-19')
            entry['access_attempts'] = attempts
            return {'id':ident,'status':'downloaded','entry':entry}
        except Exception as exc:
            attempts.append({'url':url,'error':str(exc)})
    entry.update(pdf_status='access-unresolved', access_attempts=attempts)
    return {'id':ident,'status':'access-unresolved','entry':entry}

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('index',type=Path)
    parser.add_argument('--report',type=Path,required=True)
    args=parser.parse_args()
    refs=json.loads(args.index.read_text())
    manifest=yaml.safe_load((ROOT/'manifest.yaml').read_text())
    existing={p['key']:p for p in manifest['papers']}
    results=[]
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
        futures={pool.submit(archive,r,existing):r['id'] for r in refs}
        for future in concurrent.futures.as_completed(futures):
            result=future.result()
            results.append(result)
            print(result['id'],result['status'],flush=True)
    for result in sorted(results,key=lambda r:r['id']):
        if 'entry' in result:
            existing[result['entry']['key']]=result['entry']
    manifest['papers']=list(existing.values())
    (ROOT/'manifest.yaml').write_text(yaml.safe_dump(manifest,allow_unicode=True,sort_keys=False))
    args.report.parent.mkdir(parents=True,exist_ok=True)
    args.report.write_text(json.dumps(sorted(results,key=lambda r:r['id']),ensure_ascii=False,indent=2)+'\n')

if __name__ == '__main__':
    main()
