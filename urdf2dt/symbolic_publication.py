"""Self-contained typesetting source, never a numerical simplification pass."""


def tex_escape(text):
    special = {'\\': r'\textbackslash{}', '&': r'\&', '%': r'\%', '$': r'\$',
               '#': r'\#', '_': r'\_', '{': r'\{', '}': r'\}', '~': r'\textasciitilde{}', '^': r'\textasciicircum{}'}
    return ''.join(special.get(c, c) for c in str(text))


def publication_latex(data, context, expanded=False):
    lines = [r'\documentclass[11pt,a4paper]{article}', r'\usepackage[margin=22mm]{geometry}',
             r'\usepackage{fontspec}', r'\usepackage{amsmath,amssymb}', r'\usepackage{breqn}',
             r'\setlength{\parindent}{0pt}', r'\begin{document}',
             r'\section*{URDF2DT symbolic ' + ('Jacobian' if data['label'] == 'J' else 'forward kinematics') + '}',
             tex_escape(context) + r'\par',
             tex_escape('Expanded equations' if expanded else 'Compact equations with ordered reusable definitions') + r'\par',
             tex_escape('Constants retain stored precision. No tolerance snapping, dropped terms or global simplification. Expansion removes aliases only.') + r'\par',
             tex_escape('Geometric Jacobian: target tip origin; linear then angular rows; angular velocity is not RPY rates.'
                        if data['label'] == 'J' else 'Transform maps selected tip coordinates into selected base coordinates; position in metres, rotation unitless.') + r'\par',
             r'\subsection*{Joint coordinates}', r'\begin{itemize}']
    lines += [r'\item ' + tex_escape(key) for key in data['joint_key']]
    lines += [r'\end{itemize}', r'\subsection*{Equations}']
    if expanded:
        for i, row in enumerate(data['expanded_latex']):
            for j, formula in enumerate(row):
                if formula is None:
                    raise ValueError('An entry exceeds the expansion limit. Export compact equations instead.')
                lines += [r'\begin{dmath*}' + data['label'] + '_{' + f'{i+1},{j+1}' + '}(q) = ' + formula + r'\end{dmath*}']
    else:
        lines += [r'\[' + data['label'] + '(q) = ' + data['matrix_latex'] + r'\]']
        lines += [r'\begin{dmath*}' + symbol + ' = ' + formula + r'\end{dmath*}'
                  for symbol, formula in data['definition_latex']]
    lines += [r'\end{document}', '% Compile with LuaLaTeX (fontspec and breqn required).']
    return '\n'.join(lines) + '\n'
