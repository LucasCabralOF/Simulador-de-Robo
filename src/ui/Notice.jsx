export function Notice({ variant = 'warning', role, as: Tag = 'div', className = '', children }) {
  const classes = [
    'ui-notice',
    'notice',
    variant !== 'warning' && variant,
    className,
  ].filter(Boolean).join(' ');

  return (
    <Tag className={classes} role={role}>
      {children}
    </Tag>
  );
}
