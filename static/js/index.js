function toggleMoreWorks() {
  const dropdown = document.getElementById("moreWorksDropdown");
  const button = document.querySelector(".more-works-btn");

  if (!dropdown || !button) return;

  const isOpen = dropdown.classList.contains("show");
  dropdown.classList.toggle("show", !isOpen);
  button.classList.toggle("active", !isOpen);
}

function closeMoreWorks() {
  const dropdown = document.getElementById("moreWorksDropdown");
  const button = document.querySelector(".more-works-btn");

  if (!dropdown || !button) return;

  dropdown.classList.remove("show");
  button.classList.remove("active");
}

function copyBibTeX() {
  const bibtexElement = document.getElementById("bibtex-code");
  const button = document.querySelector(".copy-bibtex-btn");
  const copyText = button ? button.querySelector(".copy-text") : null;

  if (!bibtexElement || !button || !copyText) return;

  const setCopied = () => {
    button.classList.add("copied");
    copyText.textContent = "Copied";

    setTimeout(() => {
      button.classList.remove("copied");
      copyText.textContent = "Copy";
    }, 1800);
  };

  if (navigator.clipboard) {
    navigator.clipboard.writeText(bibtexElement.textContent).then(setCopied).catch(() => {
      fallbackCopy(bibtexElement.textContent);
      setCopied();
    });
  } else {
    fallbackCopy(bibtexElement.textContent);
    setCopied();
  }
}

function fallbackCopy(text) {
  const textArea = document.createElement("textarea");
  textArea.value = text;
  textArea.setAttribute("readonly", "");
  textArea.style.position = "absolute";
  textArea.style.left = "-9999px";
  document.body.appendChild(textArea);
  textArea.select();
  document.execCommand("copy");
  document.body.removeChild(textArea);
}

function scrollToTop() {
  window.scrollTo({
    top: 0,
    behavior: "smooth"
  });
}

function updateScrollState() {
  const scrollButton = document.querySelector(".scroll-to-top");
  const nav = document.querySelector(".nav");
  const progress = document.querySelector(".progress-bar");
  const scrollTop = window.pageYOffset || document.documentElement.scrollTop;
  const docHeight = document.documentElement.scrollHeight - window.innerHeight;

  if (scrollButton) {
    scrollButton.classList.toggle("visible", scrollTop > 300);
  }

  if (nav) {
    nav.classList.toggle("is-scrolled", scrollTop > 8);
  }

  if (progress) {
    const width = docHeight > 0 ? (scrollTop / docHeight) * 100 : 0;
    progress.style.width = `${Math.min(100, Math.max(0, width))}%`;
  }
}

function setupBenchmarkTabs() {
  const tabs = Array.from(document.querySelectorAll("[data-benchmark-target]"));
  const panels = Array.from(document.querySelectorAll("[data-benchmark-panel]"));

  if (!tabs.length || !panels.length) return;

  const setActiveBenchmark = (target, shouldFocus = false) => {
    tabs.forEach((tab) => {
      const isActive = tab.dataset.benchmarkTarget === target;
      tab.classList.toggle("is-active", isActive);
      tab.setAttribute("aria-selected", String(isActive));
      tab.tabIndex = isActive ? 0 : -1;

      if (isActive && shouldFocus) {
        tab.focus();
      }
    });

    panels.forEach((panel) => {
      const isActive = panel.dataset.benchmarkPanel === target;
      panel.hidden = !isActive;
    });
  };

  tabs.forEach((tab, index) => {
    tab.addEventListener("click", () => {
      setActiveBenchmark(tab.dataset.benchmarkTarget);
    });

    tab.addEventListener("keydown", (event) => {
      if (!["ArrowLeft", "ArrowRight", "Home", "End"].includes(event.key)) return;

      event.preventDefault();
      let nextIndex = index;

      if (event.key === "ArrowLeft") {
        nextIndex = (index - 1 + tabs.length) % tabs.length;
      } else if (event.key === "ArrowRight") {
        nextIndex = (index + 1) % tabs.length;
      } else if (event.key === "Home") {
        nextIndex = 0;
      } else if (event.key === "End") {
        nextIndex = tabs.length - 1;
      }

      setActiveBenchmark(tabs[nextIndex].dataset.benchmarkTarget, true);
    });
  });

  const activeTab = tabs.find((tab) => tab.classList.contains("is-active")) || tabs[0];
  setActiveBenchmark(activeTab.dataset.benchmarkTarget);
}

function setupReveal() {
  const revealNodes = document.querySelectorAll(".reveal");

  if (!("IntersectionObserver" in window)) {
    revealNodes.forEach((node) => node.classList.add("is-visible"));
    return;
  }

  const observer = new IntersectionObserver((entries) => {
    entries.forEach((entry) => {
      if (entry.isIntersecting) {
        entry.target.classList.add("is-visible");
        observer.unobserve(entry.target);
      }
    });
  }, { threshold: 0.12 });

  revealNodes.forEach((node) => observer.observe(node));
}

function revealHashTarget() {
  const hash = window.location.hash.replace("#", "");
  if (!hash) return;

  const target = document.getElementById(decodeURIComponent(hash));
  if (!target) return;

  if (target.classList.contains("reveal")) {
    target.classList.add("is-visible");
  }

  target.querySelectorAll(".reveal").forEach((node) => {
    node.classList.add("is-visible");
  });
}

document.addEventListener("click", (event) => {
  const container = document.querySelector(".more-works-container");
  if (container && !container.contains(event.target)) {
    closeMoreWorks();
  }
});

document.addEventListener("keydown", (event) => {
  if (event.key === "Escape") {
    closeMoreWorks();
  }
});

window.addEventListener("scroll", updateScrollState);
window.addEventListener("resize", updateScrollState);
window.addEventListener("hashchange", revealHashTarget);

document.addEventListener("DOMContentLoaded", () => {
  updateScrollState();
  setupReveal();
  revealHashTarget();
  setupBenchmarkTabs();
});
