
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import {Component, Input} from '@angular/core';

import * as Viz from 'viz.js/viz-lite.js';

@Component({
  selector: 'dotgraph',
  templateUrl: './template.html',
})
export class DotGraphComponent {
  @Input()  dot: string;
  @Input()  strip_size?: boolean = true;
  @Input()  css_class?: string;

  render_graph()
  {
    let svg = Viz(this.dot);
    let parser = new DOMParser();
    let xml = parser.parseFromString(svg, "text/xml");
    //xml = xml.getElementsByTagName("svg")[0];

    if (this.strip_size) {
      xml.getElementsByTagName("svg")[0].removeAttribute("width");
      xml.getElementsByTagName("svg")[0].removeAttribute("height");
    }
    if (this.css_class) {
      xml.getElementsByTagName("svg")[0].setAttribute("class", this.css_class);
    }
    return new XMLSerializer().serializeToString(xml);
  }
}
