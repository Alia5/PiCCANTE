<!--
 PiCCANTE - PiCCANTE Car Controller Area Network Tool for Exploration
 Copyright (C) 2025 Peter Repukat

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 -->
<script lang="ts">
import { page } from '$app/state';

const commitLink = $derived.by(() => {
    const splitted = page.data.version.split('-');
    const commit = splitted[splitted.length - 1];
    return `https://github.com/Alia5/PiCCANTE/commit/${commit}`;
});
const versionWithoutCommit = $derived.by(() => {
    const splitted = page.data.version.split('-');
    return splitted.slice(0, -1).join('-');
});
</script>

<div class="card">
    <h2>Build Info</h2>
    <b>Board</b><span>{page.data.board}</span>
    <b>Version</b><span
        >{versionWithoutCommit}-<a href={commitLink} target="_blank">{page.data.version.split('-').pop()}</a
        ></span>
    <b>Build Date</b><span>{page.data.build_date} - {page.data.build_time}</span>
    <a href="https://github.com/Alia5/PiCCANTE" target="_blank">PiCCANTE on GitHub</a>
</div>

<style>
.card {
    padding: 1em;
    place-self: center;
    display: grid;
    column-gap: 1.2em;
    row-gap: 0.5em;
    grid-template-columns: min-content auto;
    & > :first-child {
        grid-column: span 2;
    }
    & > :last-child {
        padding-top: 1em;
        grid-column: span 2;
    }
    & b {
        white-space: nowrap;
    }
}
</style>
