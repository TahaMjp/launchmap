// Copyright (c) 2025 Kodo Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import path from 'path';
import http from 'http';
import serveHandler from 'serve-handler';

export class TestServer {
  constructor(port = 3000, publicDir = path.join(__dirname, '../../../webview')) {
    this.port = port;
    this.publicDir = publicDir;
    this.server = null;
  }

  async start() {
    this.server = http.createServer((req, res) =>
      serveHandler(req, res, { public: this.publicDir })
    );
    await new Promise((resolve) => this.server.listen(this.port, resolve));
    console.log(`✅ Test server running at http://localhost:${this.port}`);
  }

  async stop() {
    if (!this.server) return;
    await new Promise((resolve) => this.server.close(resolve));
    console.log('🛑 Test server stopped');
  }

  get url() {
    return `http://localhost:${this.port}/tests/assets/index.html`;
  }
}
