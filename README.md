# Simulador de cinematica DH

React/Vite e Three.js com persistencia Express/SQLite. Cobre os assuntos das aulas de 02/12/2021, 09/12/2021, 14/12/2021, 01/02/2022 e 03/02/2022 fornecidas como referencia.

## Executar

```sh
npm install
npm run dev
```

O comando inicia API e interface juntas. A interface usa http://127.0.0.1:5173 (ou a proxima porta livre); a API usa 3001. `npm run dev:web` e `npm run dev:api` iniciam cada servico separadamente. Os servidores ficam restritos ao computador local.

Variaveis opcionais:

- `PORT`: porta da API; o proxy Vite acompanha o mesmo valor.
- `API_PROXY_TARGET`: endereco completo de outra API para o proxy de desenvolvimento/preview.
- `VITE_API_URL`: URL base incluindo `/api`, incorporada durante o build. Padrao: `/api`, na mesma origem.
- `DB_PATH`: caminho do banco SQLite; padrao `database.sqlite` na raiz. Testes usam `:memory:` e nao alteram o banco pessoal.

Para producao: `npm run build`, servir `dist` e encaminhar `/api` ao processo `npm run start:api`. Para inspecao local do build, iniciar a API e executar `npm run preview`.

## Recursos

| Aba | Conteudo |
| --- | --- |
| Modelo | Juntas R/P, tabela DH, limites, atuadores, salvar/carregar/apagar |
| Direta | Origens globais, A_i, T_0_i, T_0_n, rotacao, RPY e eixo-angulo |
| Diferencial | Jacobiano, SVD, posto, condicionamento, velocidades diretas/inversas, integracao limitada |
| Inversa | Alvos XY, XYZ ou pose; solver limitado, residuos e aplicacao explicita |

## Convencoes

- DH padrao: `A_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)`; `T_0_n = A_1 ... A_n`.
- Convencao visual: motores R ficam ancorados nas origens O_(i-1) de cada junta, de onde partem os deslocamentos axiais d_i e normais a_i. O motor da base permanece ancorado no referencial 0. As conexoes permanecem continuas para qualquer valor de d_i.
- `theta`, `alpha`, limites R e posicoes R usam graus na interface. Trigonometria usa radianos. Velocidades R usam **rad/s**, inclusive na inversa diferencial; P usa **u/s**.
- `u` e uma unidade consistente de comprimento definida pelo modelo. Nao misturar metros e milimetros na mesma tabela.
- RPY: `R = Rz(yaw) Ry(pitch) Rx(roll)`. O bloqueio de gimbal informa a nao unicidade. O eixo-angulo usa quaternion, tratando rotacao nula e 180 graus.
- Jacobiano na base global: R tem coluna `[z_(i-1) x (p_n-p_(i-1)); z_(i-1)]`; P tem `[z_(i-1); 0]`. `v = [vx, vy, vz, wx, wy, wz] = J qdot`. Velocidade angular nao e a derivada dos angulos RPY.
- SVD e condicionamento usam `J_normalizado = W J S`: W divide linhas lineares pelo comprimento de referencia L; S multiplica colunas P por L. Assim, converter coerentemente as unidades nao altera o diagnostico. A matriz exibida e o Jacobiano original; seu determinante so aparece para uma tarefa quadrada.
- Posto numerico: limiar `max(1e-12, sigma_max * 1e-8)`. Comparamos com `min(dimensoes da tarefa, numero de juntas)` e mostramos separadamente a controlabilidade de toda a tarefa. Posto reduzido pode indicar singularidade ou restricao estrutural. Sem determinar o posto maximo do mecanismo inteiro, nao afirmamos qual. Um Jacobiano 6x3 de posto 3 nao e automaticamente singular.
- Inversa diferencial: SVD de ml-matrix com ganho `sigma/(sigma^2+lambda^2)`. Lambda zero produz pseudoinversa truncada. Mostramos velocidade obtida e residuo, inclusive para comandos incompativeis. Integracao satura nos limites e informa as juntas afetadas.
- Inversa de posicao/pose: iteracoes amortecidas, busca de passo e quatro sementes deterministicas, ate 160 iteracoes por semente. Posicao normalizada por L; erro angular como vetor de rotacao na base global. Convergencia exige norma do erro da tarefa <= `1e-5`. Em XY, Z e orientacao nao sao impostos. Resultados sem convergencia nao podem ser aplicados. Falha numerica nao prova impossibilidade do alvo.
- Validacao: 1 a 24 juntas, numeros finitos de modulo <= 1e6, tipos R/P, IDs unicos, limites ordenados e posicoes dentro dos limites. Modelos antigos sem velocidades recebem zero. Exibimos `det(R)` e `||R^T R-I||`. Isso valida a cadeia numerica, mas nao certifica referenciais de um robo real, colisoes, resistencia mecanica ou dinamica.

## RPR inicial

Preservado o modelo existente: `alpha1=90`, `alpha2=-90`, `a3=5`. Esta tabela descreve movimento de posicao planar XY:

```text
x = a3*cos(theta1+theta3) + d2*sin(theta1)
y = a3*sin(theta1+theta3) - d2*cos(theta1)
z = 0
yaw = theta1+theta3
```

Em `(45 graus, 5 u, 30 graus)`, a posicao e `(4.829629, 1.294095, 0)` u e yaw e 75 graus. Por isso a tarefa inicial e XY. Uma montagem com P vertical exige outra tabela DH, mesmo tendo sequencia R-P-R.

## Validacao

```sh
npm test
npm run lint
npm run build
npx playwright install chromium
npm run test:e2e
```

No Windows com Edge, pode-se definir `$env:PLAYWRIGHT_CHANNEL='msedge'` antes dos testes de navegador. Para testar producao, fazer o build e definir `$env:E2E_PRODUCTION='1'`.

Testes cobrem DH, diferencas finitas do Jacobiano linear/angular, singularidades, unidades, orientacoes degeneradas, solvers, limites e API com SQLite isolado. Playwright verifica desktop/celular, pixels WebGL, orbita, movimento, persistencia e falhas da API. Capturas ficam em `artifacts/browser`.

## Referencias complementares

- [ml-matrix: implementacao de SVD](https://github.com/mljs/matrix).
- [Modern Robotics: singularidades e posto do Jacobiano](https://modernrobotics.northwestern.edu/nu-gm-book-resource/5-3-singularities/).
