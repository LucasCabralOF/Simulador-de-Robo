import pypdf
import os

pdf_files = [f for f in os.listdir('documentação') if f.endswith('.pdf')]

for pdf_name in pdf_files:
    pdf_path = os.path.join('documentação', pdf_name)
    try:
        reader = pypdf.PdfReader(pdf_path)
        print(f"{pdf_name}: {len(reader.pages)} pages")
        for i, page in enumerate(reader.pages):
            for img_name in page.images:
                # check image size or save small thumbnail
                pass
    except Exception as e:
        print(f"Error {pdf_name}: {e}")
