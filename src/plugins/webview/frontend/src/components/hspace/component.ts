
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import {Component, Input} from '@angular/core';

@Component({
  selector: 'ff-hspace',
  templateUrl: './template.html',
})
export class HSpaceComponent {
  @Input() width: number;
}
