
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// Copyright  2017  The Kubernetes Authors
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import {Component, Input} from '@angular/core';

@Component({
  selector: 'ff-list-zero-state',
  templateUrl: './template.html',
  styleUrls: ['./style.scss']
})
export class ListZeroStateComponent
{
  @Input() message: string = "No resources found.";
}
