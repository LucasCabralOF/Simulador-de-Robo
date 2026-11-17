export function Button({
  variant = 'default',
  icon = false,
  block = false,
  type = 'button',
  className = '',
  children,
  ...props
}) {
  const classes = [
    'ui-btn',
    variant === 'primary' && 'ui-btn-primary',
    variant === 'danger' && 'ui-btn-danger',
    icon && 'ui-btn-icon',
    block && 'ui-btn-block',
    className,
  ].filter(Boolean).join(' ');

  return (
    <button type={type} className={classes} {...props}>
      {children}
    </button>
  );
}

export function FileUploadButton({
  accept,
  onChange,
  inputAriaLabel,
  ariaLabel,
  title,
  icon = false,
  className = '',
  children,
}) {
  const classes = ['ui-btn', icon && 'ui-btn-icon', className].filter(Boolean).join(' ');
  return (
    <label className={classes} title={title} aria-label={ariaLabel}>
      {children}
      <input
        type="file"
        accept={accept}
        className="visually-hidden"
        onChange={onChange}
        aria-label={inputAriaLabel}
      />
    </label>
  );
}
