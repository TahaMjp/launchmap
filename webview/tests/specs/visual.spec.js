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

import { test, expect } from '@playwright/test';
import fs from 'fs';
import path from 'path';

const launchFilesDir = path.resolve(__dirname, '../../../parser/tests/real_cases/expected_outputs');

test.describe("LaunchMap Visual Tests", () => {
    const files = fs.readdirSync(launchFilesDir).filter(f => f.endsWith(".json"));

    for (const file of files) {
        test(`renders correctly for ${file}`, async ({ page }) => {
            await page.goto(process.env.TEST_SERVER_URL);

            const launchData = JSON.parse(
                fs.readFileSync(path.join(launchFilesDir, file), "utf8")
            );

            await page.evaluate((data) => {
                window.postMessage({ type: 'launchmap-data', data }, '*');
            }, launchData);

            await page.waitForSelector(".block");
            await expect(page).toHaveScreenshot(`${file}.png`, { 
                fullPage: true,
                maxDiffPixelRatio: 0.01 
            });
        });
    }
});