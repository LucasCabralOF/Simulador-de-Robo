import fitz # PyMuPDF
import glob
import os

pdf_files = glob.glob('documentação/*.pdf')
print('Found PDFs:', pdf_files)

for pdf_path in pdf_files:
    doc = fitz.open(pdf_path)
    print(f'\n=== {pdf_path} ({len(doc)} pages) ===')
    text_sample = ""
    for i in range(min(5, len(doc))):
        text_sample += doc[i].get_text()
    if text_sample.strip():
        print(f"Has text (sample length {len(text_sample)}):")
        # Search for keywords
        for page_num in range(len(doc)):
            page_text = doc[page_num].get_text()
            for kw in ['junta', 'cilindro', 'torção', 'roll', 'pitch', 'yaw', 'punho', 'grau de liberdade', 'mobilidade']:
                if kw in page_text.lower():
                    print(f"  Page {page_num+1} matches '{kw}': {page_text[:120].strip()}...")
                    break
    else:
        print("Scanned / Image-only PDF")
