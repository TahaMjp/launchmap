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
import { SAMPLE_DATA, COMPONENT_SELECTORS } from '../fixtures/sample-data';

test.describe("Component Drag Tests", () => {
    COMPONENT_SELECTORS.forEach(({ type, selector }) => {
        test(`dragging ${type} works`, async ({ page }) => {
            await page.goto(process.env.TEST_SERVER_URL);

            await page.evaluate((data) => {
                window.postMessage({ type: 'launchmap-data', data }, '*');
            }, SAMPLE_DATA);

            const block = page.locator(selector).first();
            const boxBefore = await block.boundingBox();

            await block.dragTo(page.locator('body'), {
                targetPosition: { x: 300, y: 200 },
            });

            const boxAfter = await block.boundingBox();
            expect(boxAfter.x).not.toBe(boxBefore.x);
            expect(boxAfter.y).not.toBe(boxBefore.y);

            await expect(page).toHaveScreenshot(`${type}-drag.png`, {
                fullPage: true,
                maxDiffPixelRatio: 0.02
            });
        });
    });
});