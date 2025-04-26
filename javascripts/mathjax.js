// docs/javascripts/mathjax.js
window.MathJax = {
  tex: {
    inlineMath: [['$', '$'], ['\\(', '\\)']], // 行内公式标识符
    displayMath: [['$$', '$$'], ['\\[', '\\]']], // 块级公式标识符
    processEscapes: true,
    processEnvironments: true,
    packages: {'[+]': ['bm']} // <--- 关键：添加 'bm' 宏包
  },
  options: {
    ignoreHtmlClass: '.*|', // 忽略特定 HTML 类
    processHtmlClass: 'arithmatex' // 处理特定 HTML 类
  }
};


  