export function Navbar({ items, activeId, onChange, ariaLabel = 'Navegação' }) {
  return (
    <nav className="ui-navbar" aria-label={ariaLabel}>
      {items.map(item => (
        <button
          key={item.id}
          type="button"
          className="ui-navbar-tab"
          aria-current={activeId === item.id ? 'page' : undefined}
          onClick={() => onChange(item.id)}
        >
          {item.label}
        </button>
      ))}
    </nav>
  );
}

export function SegmentedTabs({ items, activeId, onChange, ariaLabel }) {
  return (
    <div className="ui-segmented" role="group" aria-label={ariaLabel}>
      {items.map(item => (
        <button
          key={item.id}
          type="button"
          className="ui-segmented-item"
          aria-current={activeId === item.id ? 'page' : undefined}
          onClick={() => onChange(item.id)}
        >
          {item.label}
        </button>
      ))}
    </div>
  );
}
