import { useState } from 'react';
import { ChevronDown, ChevronRight } from 'lucide-react';
import { Button } from '../ui';

export default function CollapsibleSection({
  title,
  collapsible = true,
  defaultOpen = true,
  open: controlledOpen,
  onToggle,
  actions = null,
  children,
}) {
  const [uncontrolledOpen, setUncontrolledOpen] = useState(defaultOpen);
  const isControlled = controlledOpen !== undefined;
  const isOpen = collapsible ? (isControlled ? controlledOpen : uncontrolledOpen) : true;

  const handleToggle = () => {
    const next = !isOpen;
    if (!isControlled) setUncontrolledOpen(next);
    onToggle?.(next);
  };

  return (
    <section className={collapsible ? `section-collapsible${isOpen ? '' : ' is-collapsed'}` : undefined}>
      <div className="section-title">
        <div className="section-heading-group">
          {collapsible && (
            <Button
              icon
              className="section-toggle-btn"
              title={isOpen ? `Recolher ${title}` : `Expandir ${title}`}
              aria-label={isOpen ? `Recolher ${title}` : `Expandir ${title}`}
              aria-expanded={isOpen}
              onClick={handleToggle}
            >
              {isOpen ? <ChevronDown size={16} /> : <ChevronRight size={16} />}
            </Button>
          )}
          <h2>{title}</h2>
        </div>
        {actions && <div className="section-header-actions">{actions}</div>}
      </div>
      {isOpen && children}
    </section>
  );
}
