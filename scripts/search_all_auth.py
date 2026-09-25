import fitz
import os

pdf_files = [f for f in os.listdir('documentação') if f.endswith('.pdf')]

for pdf_name in pdf_files:
    path = os.path.join('documentação', pdf_name)
    doc = fitz.open(path)
    if doc.is_encrypted:
        doc.authenticate('')
    print(f"\n=== {pdf_name} ({len(doc)} pages) ===")
    for p in range(len(doc)):
        text = doc[p].get_text()
        for kw in ['cilindr', 'revolu', 'torção', 'rotat', 'roll', 'twist', 'prismat', 'junta']:
            if kw in text.lower():
                print(f"  P{p+1} [{kw}]: {text[:100].strip().replace(chr(10), ' ')}")
                break
