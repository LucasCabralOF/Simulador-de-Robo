import pypdf
import glob

pdf_files = glob.glob('documentação/*.pdf')
for path in pdf_files:
    try:
        reader = pypdf.PdfReader(path)
        print(f"\n=== {path} ({len(reader.pages)} pages) ===")
        found = False
        for idx, page in enumerate(reader.pages):
            text = page.extract_text() or ""
            for kw in ['junta', 'revoluta', 'prismática', 'torção', 'cilíndrica', 'esférica', 'roll', 'denavit', 'hartenberg', 'par cinemático', 'grau de liberdade']:
                if kw in text.lower():
                    print(f"  Page {idx+1} [{kw}]: {text[:140].strip().replace(chr(10), ' ')}...")
                    found = True
                    break
        if not found:
            print("  (No keyword matches or image-based)")
    except Exception as e:
        print(f"  Error reading {path}: {e}")
