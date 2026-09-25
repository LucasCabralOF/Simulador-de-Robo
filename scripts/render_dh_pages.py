import pypdf
import fitz

doc = fitz.open('documentação/Aula_DH.pdf')
for p in [13, 14, 15, 19, 20, 27, 28, 29]: # 0-indexed
    doc[p].get_pixmap(dpi=150).save(f'artifacts/auladh_p{p+1}.png')
print("Rendered Aula_DH pages")
