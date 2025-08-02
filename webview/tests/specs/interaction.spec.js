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

test.describe("Zoom, Pan, and Drag interactions with visualization state checks", () => {
    const files = fs.readdirSync(launchFilesDir).filter(f => f.endsWith(".json"));

    for (const file of files) {
        test(`state consistent for ${file}`, async ({ page }) => {
            await page.goto(process.env.TEST_SERVER_URL);

            const launchData = JSON.parse(
                fs.readFileSync(path.join(launchFilesDir, file), "utf8")
            );

            await page.evaluate((data) => {
                window.postMessage({ type: 'launchmap-data', data }, '*');
            }, launchData);

            await page.waitForSelector(".block");
            
            // Capture initial edge positions
            const edgesBefore = await page.$$eval('#edge-layer path', paths =>
                paths.map(p => p.getAttribute('d'))
            );
            if (edgesBefore.length === 0) {
                console.warn(`⚠️ No edges found in ${file}, skipping edge checks.`);
            }

            // Drag Test
            const block = page.locator('.block-header').first();
            const boxBefore = await block.boundingBox();

            await block.dragTo(page.locator('body'), {
                targetPosition: { x: 300, y: 200 },
            });

            const boxAfter = await block.boundingBox();
            expect(boxAfter.x).not.toBe(boxBefore.x);
            expect(boxAfter.y).not.toBe(boxBefore.y);

            const edgesAfterDrag = await page.$$eval('#edge-layer path', paths =>
                paths.map(p => p.getAttribute('d'))
            );
            if (edgesBefore.length > 0) {
                expect(edgesAfterDrag).not.toEqual(edgesBefore);
            }

            // Pan test
            const zoomLayer = page.locator('#zoom-layer');
            const transformBeforePan = await zoomLayer.evaluate(el => el.style.transform);

            await page.keyboard.down('Shift');
            await page.mouse.move(300, 300);
            await page.mouse.down();
            await page.mouse.move(500, 400);
            await page.mouse.up();
            await page.keyboard.up('Shift');

            const transformAfterPan = await zoomLayer.evaluate(el => el.style.transform);
            expect(transformAfterPan).not.toBe(transformBeforePan);

            const edgesAfterPan = await page.$$eval('#edge-layer path', paths =>
                paths.map(p => p.getAttribute('d'))
            );
            if (edgesBefore.length > 0) {
                expect(edgesAfterPan).not.toEqual(edgesAfterDrag);
            }

            // Zoom Test
            const transformBeforeZoom = await zoomLayer.evaluate(el => el.style.transform);

            await page.mouse.wheel(0, -200);
            const transformAfterZoomIn = await zoomLayer.evaluate(el => el.style.transform);
            expect(transformAfterZoomIn).not.toBe(transformBeforeZoom);

            await page.mouse.wheel(0, 200);
            const transformAfterZoomOut = await zoomLayer.evaluate(el => el.style.transform);
            expect(transformAfterZoomOut).not.toBe(transformAfterZoomIn);

            const edgesAfterZoom = await page.$$eval('#edge-layer path', paths =>
                paths.map(p => p.getAttribute('d'))
            );
            if (edgesBefore.length > 0) {
                expect(edgesAfterZoom).not.toEqual(edgesAfterPan);
            }

            // Final Visual Snapshot
            await expect(page).toHaveScreenshot(`${file}-final.png`, {
                fullPage: true,
                maxDiffPixelRatio: 0.02
            });
        });
    }
});