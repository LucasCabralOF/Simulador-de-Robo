import { useEffect, useId, useLayoutEffect, useRef, useState } from 'react';
import { createPortal } from 'react-dom';
import { Check, ChevronDown } from 'lucide-react';

export function Select({
  options,
  value,
  onChange,
  placeholder = 'Selecionar...',
  ariaLabel,
  compact = false,
  disabled = false,
  className = '',
}) {
  const [open, setOpen] = useState(false);
  const [activeIndex, setActiveIndex] = useState(-1);
  const [menuStyle, setMenuStyle] = useState(null);
  const triggerRef = useRef(null);
  const menuRef = useRef(null);
  const listboxId = useId();

  const selectedIndex = options.findIndex(opt => opt.value === value);
  const selectedOption = selectedIndex >= 0 ? options[selectedIndex] : null;

  useLayoutEffect(() => {
    if (!open || !triggerRef.current) return;
    const rect = triggerRef.current.getBoundingClientRect();
    const margin = 8;
    const desiredWidth = compact ? Math.max(rect.width, 84) : rect.width;
    const width = Math.min(desiredWidth, window.innerWidth - margin * 2);
    const left = Math.max(margin, Math.min(rect.left, window.innerWidth - width - margin));
    const estHeight = Math.min(264, options.length * 36 + 12);
    const spaceBelow = window.innerHeight - rect.bottom - margin;
    const spaceAbove = rect.top - margin;
    const openUp = spaceBelow < estHeight && spaceAbove > spaceBelow;
    const maxHeight = Math.max(110, Math.min(264, openUp ? spaceAbove : spaceBelow));

    setMenuStyle(
      openUp
        ? { left, width, bottom: window.innerHeight - rect.top + 4, maxHeight }
        : { left, width, top: rect.bottom + 4, maxHeight }
    );
  }, [open, compact, options.length]);

  useEffect(() => {
    if (!open) return;
    const handlePointerDown = (e) => {
      if (triggerRef.current?.contains(e.target) || menuRef.current?.contains(e.target)) return;
      setOpen(false);
    };
    const handleScroll = (e) => {
      if (menuRef.current?.contains(e.target)) return;
      setOpen(false);
    };
    const handleResize = () => setOpen(false);
    document.addEventListener('pointerdown', handlePointerDown);
    window.addEventListener('scroll', handleScroll, true);
    window.addEventListener('resize', handleResize);
    return () => {
      document.removeEventListener('pointerdown', handlePointerDown);
      window.removeEventListener('scroll', handleScroll, true);
      window.removeEventListener('resize', handleResize);
    };
  }, [open]);

  useEffect(() => {
    if (!open || activeIndex < 0 || !menuRef.current) return;
    const item = menuRef.current.children[activeIndex];
    if (item && typeof item.scrollIntoView === 'function') {
      item.scrollIntoView({ block: 'nearest' });
    }
  }, [open, activeIndex]);

  function openMenu() {
    if (disabled || options.length === 0) return;
    const firstEnabled = options.findIndex(opt => !opt.disabled);
    setActiveIndex(selectedIndex >= 0 ? selectedIndex : firstEnabled);
    setOpen(true);
  }

  function selectOption(opt) {
    if (!opt || opt.disabled) return;
    setOpen(false);
    if (opt.value !== value) {
      onChange?.(opt.value);
    }
    triggerRef.current?.focus();
  }

  function moveActive(delta) {
    if (options.length === 0) return;
    let idx = activeIndex;
    for (let step = 0; step < options.length; step++) {
      idx = (idx + delta + options.length) % options.length;
      if (!options[idx].disabled) {
        setActiveIndex(idx);
        return;
      }
    }
  }

  function handleKeyDown(e) {
    if (disabled) return;
    if (!open) {
      if (e.key === 'ArrowDown' || e.key === 'ArrowUp' || e.key === 'Enter' || e.key === ' ') {
        e.preventDefault();
        openMenu();
      }
      return;
    }

    if (e.key === 'ArrowDown') {
      e.preventDefault();
      moveActive(1);
    } else if (e.key === 'ArrowUp') {
      e.preventDefault();
      moveActive(-1);
    } else if (e.key === 'Home') {
      e.preventDefault();
      const first = options.findIndex(opt => !opt.disabled);
      if (first >= 0) setActiveIndex(first);
    } else if (e.key === 'End') {
      e.preventDefault();
      for (let i = options.length - 1; i >= 0; i--) {
        if (!options[i].disabled) { setActiveIndex(i); break; }
      }
    } else if (e.key === 'Enter' || e.key === ' ') {
      e.preventDefault();
      if (activeIndex >= 0 && options[activeIndex]) {
        selectOption(options[activeIndex]);
      }
    } else if (e.key === 'Escape') {
      e.preventDefault();
      setOpen(false);
      triggerRef.current?.focus();
    } else if (e.key === 'Tab') {
      setOpen(false);
    }
  }

  return (
    <>
      <button
        ref={triggerRef}
        type="button"
        role="combobox"
        aria-label={ariaLabel}
        aria-expanded={open}
        aria-haspopup="listbox"
        aria-controls={listboxId}
        aria-activedescendant={open && activeIndex >= 0 ? `${listboxId}-opt-${activeIndex}` : undefined}
        disabled={disabled}
        className={`ui-select-trigger${compact ? ' compact' : ''}${open ? ' open' : ''}${className ? ` ${className}` : ''}`}
        onClick={() => (open ? setOpen(false) : openMenu())}
        onKeyDown={handleKeyDown}
      >
        <span className={`ui-select-value${selectedOption ? '' : ' placeholder'}`}>
          {selectedOption ? selectedOption.label : placeholder}
        </span>
        <ChevronDown size={compact ? 13 : 14} className="ui-select-arrow" aria-hidden="true" />
      </button>

      {open && menuStyle && createPortal(
        <ul
          ref={menuRef}
          id={listboxId}
          role="listbox"
          aria-label={ariaLabel}
          className={`ui-select-dropdown${compact ? ' compact' : ''}`}
          style={menuStyle}
        >
          {options.map((opt, idx) => {
            const isSelected = opt.value === value;
            const isActive = idx === activeIndex;
            return (
              <li
                key={String(opt.value)}
                id={`${listboxId}-opt-${idx}`}
                role="option"
                aria-selected={isSelected}
                aria-disabled={opt.disabled || undefined}
                className={`ui-select-option${isSelected ? ' selected' : ''}${isActive ? ' active' : ''}${opt.disabled ? ' disabled' : ''}`}
                onMouseEnter={() => !opt.disabled && setActiveIndex(idx)}
                onMouseDown={e => e.preventDefault()}
                onClick={() => selectOption(opt)}
              >
                <span className="ui-select-option-label">{opt.label}</span>
                {isSelected && <Check size={14} className="ui-select-check" aria-hidden="true" />}
              </li>
            );
          })}
        </ul>,
        document.body
      )}
    </>
  );
}
