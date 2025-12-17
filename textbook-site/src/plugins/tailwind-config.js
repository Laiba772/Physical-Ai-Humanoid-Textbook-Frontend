module.exports = function tailwindPlugin(context, options) {
  return {
    name: "tailwind-plugin",
    configurePostCss(postcssOptions) {
      // Appends TailwindCSS and Autoprefixer to the PostCSS plugins
      postcssOptions.plugins.push(require("@tailwindcss/postcss"));
      postcssOptions.plugins.push(require("autoprefixer"));
      return postcssOptions;
    },
  };
};