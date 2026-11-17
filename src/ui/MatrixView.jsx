const defaultFmt = (n, digits = 4) => !Number.isFinite(n) ? '∞' : (Math.abs(n) < 0.5 * 10 ** -digits ? 0 : n).toFixed(digits);

export function MatrixView({ label, values, headers, formatter = defaultFmt }) {
  return (
    <figure className="ui-matrix matrix-view">
      <figcaption>{label}</figcaption>
      <div className="ui-matrix-box table-scroll">
        <table aria-label={label}>
          {headers && <thead><tr>{headers.map(h => <th key={h}>{h}</th>)}</tr></thead>}
          <tbody>{values.map((row, i) => <tr key={i}>{row.map((v, j) => <td key={j}>{formatter(v)}</td>)}</tr>)}</tbody>
        </table>
      </div>
    </figure>
  );
}
