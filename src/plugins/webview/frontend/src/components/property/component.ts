
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// Copyright  2017  The Kubernetes Authors
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import {Component, Input} from '@angular/core';

@Component({
  selector: 'ff-property',
  templateUrl: './template.html',
  styleUrls: ['./style.scss'],
})
export class PropertyComponent {
  @Input() direction?: string = "horizontal";
}
