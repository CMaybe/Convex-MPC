import HtmlWebpackPlugin from "html-webpack-plugin";
import { fileURLToPath } from "node:url";
import path from "node:path";

const root = path.dirname(fileURLToPath(import.meta.url));

export default {
  entry: path.resolve(root, "src/main.jsx"),
  output: {
    path: path.resolve(root, "dist"),
    filename: "assets/[name].[contenthash].js",
    clean: true,
    publicPath: "/"
  },
  resolve: {
    extensions: [".js", ".jsx"]
  },
  module: {
    rules: [
      {
        test: /\.css$/,
        use: ["style-loader", "css-loader"]
      },
      {
        test: /\.jsx?$/,
        exclude: /node_modules/,
        use: {
          loader: "babel-loader",
          options: {
            presets: [
              ["@babel/preset-env", { targets: "defaults" }],
              ["@babel/preset-react", { runtime: "automatic" }]
            ]
          }
        }
      }
    ]
  },
  plugins: [new HtmlWebpackPlugin({ template: path.resolve(root, "index.html") })],
  devServer: {
    port: 3000,
    static: {
      directory: path.resolve(root, "public")
    },
    historyApiFallback: true,
    hot: true
  }
};
