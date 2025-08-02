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

export class LayoutManager {
    constructor(startX = 100, startY = 100, columnSpacing = 350, rowSpacing = 40) {
        this.startX = startX;
        this.startY = startY;
        this.columnSpacing = columnSpacing;
        this.rowSpacing = rowSpacing;
        this.columns = [];

        this.typeToColumn = {
            "argument": 0,
            "environment-variable": 0,
            "python-expression": 0,
            "event-handler": 1,
            "node": 2,
            "composable-node": 2,
            "include": 2,
            "group": 3,
            "timer-action": 3,
            "composable-container": 3,
            "opaque-function": 3
        };
    }

    getColumnIndexForType(type) {
        return this.typeToColumn[type] ?? 0;
    }

    ensureColumn(columnIndex) {
        // Build all missing columns up to columnIndex
        for (let i = 0; i <= columnIndex; i++) {
            if (!this.columns[i]) {
                const prev = this.columns[i-1];
                this.columns[i] = {
                    x: prev ? prev.x + prev.maxWidth + this.columnSpacing : this.startX,
                    y: this.startY,
                    maxWidth: 0,
                    blocks: []
                };
            }
        }

        return this.columns[columnIndex];
    }

    placeBlock(block, type) {
        const columnIndex = this.getColumnIndexForType(type);
        const col = this.ensureColumn(columnIndex);
        const rect = block.getBoundingClientRect();

        block.style.left = `${col.x}px`;
        block.style.top = `${col.y}px`;

        col.blocks.push(block);

        col.y += rect.height + this.rowSpacing;
        if (rect.width > col.maxWidth) {
            col.maxWidth = rect.width;
            this.reflowColumnsFrom(columnIndex + 1);
        }
    }

    reflowColumnsFrom(startIndex) {
        for (let i = startIndex; i < this.columns.length; i++) {
            const prev = this.columns[i - 1];
            const curr = this.columns[i];
            curr.x = prev.x + prev.maxWidth + this.columnSpacing;

            curr.blocks.forEach(block => {
                const rect = block.getBoundingClientRect();
                const top = parseInt(block.style.top, 10);
                block.style.left = `${curr.x}px`;
                block.style.top = `${top}px`;
            });
        }
    }

    nextColumnIndex() {
        return this.columns.length;
    }
}