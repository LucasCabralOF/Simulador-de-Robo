export const fmt = (n, digits = 4) => (!Number.isFinite(n) ? '∞' : (Math.abs(n) < 0.5 * 10 ** -digits ? 0 : n).toFixed(digits));

export const vectorText = (v, digits = 4) => v.map(n => fmt(n, digits)).join(', ');
