
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import {Component, Input, Output, EventEmitter} from '@angular/core';

import * as Viz from 'viz.js/viz.js';

@Component({
  selector: 'dot-graph',
  templateUrl: './template.html',
})
export class DotGraphComponent {
  @Input()  dot: string;
  @Input()  strip_size = true;
  @Input()  css_class?: string;

  @Output() svg_updated = new EventEmitter<string>();

  render_graph() {
    const svg = Viz(this.dot);
    const parser = new DOMParser();
    const xml = parser.parseFromString(svg, 'text/xml');
    // xml = xml.getElementsByTagName("svg")[0];

    this.svg_updated.emit(svg);

    if (this.strip_size) {
      xml.getElementsByTagName('svg')[0].removeAttribute('width');
      xml.getElementsByTagName('svg')[0].removeAttribute('height');
    }
    if (this.css_class) {
      xml.getElementsByTagName('svg')[0].setAttribute('class', this.css_class);
    }
    return new XMLSerializer().serializeToString(xml);
  }
}
