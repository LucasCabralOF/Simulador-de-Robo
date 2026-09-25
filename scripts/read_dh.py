import pypdf

reader = pypdf.PdfReader('documentação/Aula_DH.pdf')
print("Aula_DH pages:", len(reader.pages))
for i, page in enumerate(reader.pages):
    text = page.extract_text() or ""
    print(f"Page {i+1}: {text[:150].strip().replace(chr(10), ' ')}")
