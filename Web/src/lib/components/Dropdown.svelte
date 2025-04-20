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
import IcRoundArrowDropDown from '~icons/ic/round-arrow-drop-down';

// eslint-disable-next-line prefer-const
let { options, selected = $bindable() }: { options: { name: string; value: unknown }[]; selected?: unknown } =
    $props();
</script>

<div class="dropdown" style="min-width: 8em;">
    <select bind:value={selected}>
        {#each options as opt (opt.value)}
            <option value={opt.value}>{opt.name}</option>
        {/each}
    </select>
    <div class="icon">
        <IcRoundArrowDropDown style="width: 2.4em; height: 2.4em;" />
    </div>
</div>
